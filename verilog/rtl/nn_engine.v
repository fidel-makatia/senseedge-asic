// SPDX-License-Identifier: Apache-2.0
// SenseEdge - Tiny Neural Network Inference Engine
// Fully-connected: 8 inputs → 16 hidden (ReLU) → 4 outputs (argmax)
// INT8 weights and activations, single MAC unit, time-multiplexed
// Total parameters: (8*16)+16+(16*4)+4 = 212 weights/biases

`default_nettype none

module nn_engine (
    input  wire        clk,
    input  wire        rst,

    // Input interface (features from feature extraction)
    input  wire        start,
    input  wire [7:0]  feature_in,     // Feature data read port
    output reg  [2:0]  feature_addr,   // Feature address (0-7)

    // Output interface
    output reg         done,
    output reg  [1:0]  class_id,       // Classification result (0-3)
    output reg  [7:0]  confidence,     // Confidence score (max activation)
    output reg         busy,

    // Weight loading interface (via Wishbone)
    input  wire        wt_wr_en,
    input  wire [7:0]  wt_wr_addr,     // 0-211: weight address
    input  wire [7:0]  wt_wr_data      // INT8 weight value
);

    // --- Weight storage ---
    // Layer 1: 8 inputs × 16 neurons = 128 weights + 16 biases = 144
    // Layer 2: 16 inputs × 4 neurons = 64 weights + 4 biases = 68
    // Total: 212 parameters
    reg signed [7:0] weights [0:211];

    // Weight memory layout:
    // [0..127]   : Layer 1 weights (row-major: w[neuron][input])
    // [128..143] : Layer 1 biases
    // [144..207] : Layer 2 weights (row-major: w[neuron][input])
    // [208..211] : Layer 2 biases

    // --- Weight write port ---
    always @(posedge clk) begin
        if (wt_wr_en) begin
            weights[wt_wr_addr] <= wt_wr_data;
        end
    end

    // --- Hidden layer activations ---
    reg signed [15:0] hidden [0:15];   // After ReLU
    reg signed [23:0] output_act [0:3]; // Final layer output (full precision)

    // --- FSM ---
    localparam S_IDLE     = 3'd0;
    localparam S_LOAD_IN  = 3'd1;
    localparam S_LAYER1   = 3'd2;
    localparam S_RELU     = 3'd3;
    localparam S_LAYER2   = 3'd4;
    localparam S_ARGMAX   = 3'd5;
    localparam S_DONE     = 3'd6;

    reg [2:0]  state;

    // Input buffer
    reg signed [7:0] inputs [0:7];

    // MAC computation
    reg [3:0]  neuron_idx;     // Current neuron (0-15 for L1, 0-3 for L2)
    reg [3:0]  input_idx;      // Current input index
    reg signed [23:0] acc;     // Accumulator for MAC
    reg [7:0]  wt_rd_addr;     // Weight read address
    reg [3:0]  load_cnt;       // Input loading counter (needs 4 bits for 0-8)

    // Argmax
    reg signed [23:0] max_val;
    reg [1:0]  max_idx;
    reg [1:0]  argmax_cnt;
    reg        argmax_done;

    always @(posedge clk) begin
        if (rst) begin
            state        <= S_IDLE;
            done         <= 1'b0;
            busy         <= 1'b0;
            class_id     <= 2'd0;
            confidence   <= 8'd0;
            feature_addr <= 3'd0;
            neuron_idx   <= 4'd0;
            input_idx    <= 4'd0;
            load_cnt     <= 3'd0;
            argmax_done  <= 1'b0;
        end else begin
            done <= 1'b0;

            case (state)
                S_IDLE: begin
                    if (start && !wt_wr_en) begin
                        state        <= S_LOAD_IN;
                        busy         <= 1'b1;
                        load_cnt     <= 4'd0;
                        feature_addr <= 3'd0;
                    end
                end

                // --- Load 8 input features ---
                // feature_addr is combinationally read by external memory.
                // On each cycle, feature_in reflects the address set on the
                // PREVIOUS clock edge (registered output of feature_addr).
                // Cycle 0: feature_in = mem[0] (addr set in S_IDLE), set addr=1
                // Cycle 1: capture inputs[0]=mem[0], feature_in=mem[1], set addr=2
                // ...
                // Cycle 8: capture inputs[7]=mem[7], transition to L1
                S_LOAD_IN: begin
                    // Capture feature data (available from address set last cycle)
                    inputs[load_cnt[2:0]] <= feature_in;

                    if (load_cnt == 4'd7) begin
                        // All 8 inputs captured (0..7)
                        state      <= S_LAYER1;
                        neuron_idx <= 4'd0;
                        input_idx  <= 4'd0;
                        acc        <= 24'd0;
                    end else begin
                        load_cnt     <= load_cnt + 4'd1;
                        feature_addr <= load_cnt[2:0] + 3'd1;
                    end
                end

                // --- Layer 1: 8→16, MAC computation ---
                S_LAYER1: begin

                    // MAC: acc += input[input_idx] * weight[neuron*8 + input]
                    wt_rd_addr = {neuron_idx[3:0], input_idx[2:0]};  // neuron*8 + input
                    acc <= acc + (inputs[input_idx[2:0]] * weights[wt_rd_addr]);

                    if (input_idx == 4'd7) begin
                        // Done with this neuron - add bias and store
                        // Note: current MAC (input*weight) is added to acc on this cycle via line above
                        // Bias at offset 128 + neuron_idx
                        hidden[neuron_idx] <= acc
                                            + (inputs[input_idx[2:0]] * weights[wt_rd_addr])
                                            + {{16{weights[128 + neuron_idx][7]}}, weights[128 + neuron_idx]};

                        if (neuron_idx == 4'd15) begin
                            state      <= S_RELU;
                            neuron_idx <= 4'd0;
                        end else begin
                            neuron_idx <= neuron_idx + 4'd1;
                            input_idx  <= 4'd0;
                            acc        <= 24'd0;
                        end
                    end else begin
                        input_idx <= input_idx + 4'd1;
                    end
                end

                // --- ReLU activation ---
                S_RELU: begin
                    // Apply ReLU to all hidden neurons in one cycle
                    // (combinational, but registered output)
                    begin : relu_block
                        integer i;
                        for (i = 0; i < 16; i = i + 1) begin
                            if (hidden[i][15]) // Negative
                                hidden[i] <= 16'd0;
                        end
                    end
                    state      <= S_LAYER2;
                    neuron_idx <= 4'd0;
                    input_idx  <= 4'd0;
                    acc        <= 24'd0;
                end

                // --- Layer 2: 16→4, MAC computation ---
                S_LAYER2: begin
                    // MAC: acc += hidden[input_idx] * weight[144 + neuron*16 + input]
                    wt_rd_addr = 8'd144 + {neuron_idx[1:0], input_idx[3:0]};
                    acc <= acc + (hidden[input_idx] * weights[wt_rd_addr]);

                    if (input_idx == 4'd15) begin
                        // Done with this neuron - add bias (full 24-bit precision)
                        output_act[neuron_idx[1:0]] <= acc
                            + (hidden[input_idx] * weights[wt_rd_addr])
                            + {{16{weights[208 + neuron_idx[1:0]][7]}}, weights[208 + neuron_idx[1:0]]};

                        if (neuron_idx[1:0] == 2'd3) begin
                            state       <= S_ARGMAX;
                            argmax_cnt  <= 2'd0;
                            argmax_done <= 1'b0;
                            max_val     <= 24'h800000;  // Most negative (24-bit)
                            max_idx     <= 2'd0;
                        end else begin
                            neuron_idx <= neuron_idx + 4'd1;
                            input_idx  <= 4'd0;
                            acc        <= 24'd0;
                        end
                    end else begin
                        input_idx <= input_idx + 4'd1;
                    end
                end

                // --- Argmax to find winning class ---
                // Cycles 0-3: compare output_act[argmax_cnt] with max_val
                // Cycle 4 (argmax_done): latch result after last comparison registered
                S_ARGMAX: begin
                    if (!argmax_done) begin
                        if (output_act[argmax_cnt] > max_val) begin
                            max_val <= output_act[argmax_cnt];
                            max_idx <= argmax_cnt;
                        end

                        if (argmax_cnt == 2'd3) begin
                            argmax_done <= 1'b1;
                        end else begin
                            argmax_cnt <= argmax_cnt + 2'd1;
                        end
                    end else begin
                        // One cycle after last comparison — max_val/max_idx are now stable
                        class_id   <= max_idx;
                        // Confidence: saturate to 8 bits from 24-bit signed value
                        confidence <= (max_val[23]) ? 8'd0 :
                                      (max_val[23:8] != 0) ? 8'hFF : max_val[7:0];
                        state      <= S_DONE;
                    end
                end

                S_DONE: begin
                    done <= 1'b1;
                    busy <= 1'b0;
                    state <= S_IDLE;
                end

                default: state <= S_IDLE;
            endcase
        end
    end

endmodule

`default_nettype wire

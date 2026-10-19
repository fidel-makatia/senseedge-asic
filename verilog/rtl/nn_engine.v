// SPDX-License-Identifier: Apache-2.0
// SenseEdge - Tiny Neural Network Inference Engine
// Fully-connected: 8 inputs -> 16 hidden (ReLU) -> 4 outputs (argmax)
// INT8 weights, unsigned 8-bit feature inputs, single MAC unit, time-multiplexed
// Total parameters: (8*16)+16+(16*4)+4 = 212 weights/biases
//
// Corrected RTL (branch fix/audit-defects).  Changes against the fabricated revision:
//   * Feature inputs are read as UNSIGNED bytes [0,255], which is what feature_extract.v
//     emits.  The fabricated netlist read them as signed.
//   * Every product is a signed multiply, sign-extended to 24 bits before it is added.
//     In the fabricated netlist the last product of each neuron shared an expression with
//     the (unsigned) bias concatenation and was therefore evaluated unsigned.
//   * The weight registers reset to zero and can be read back one 32-bit word at a time
//     (byte k of word w is weight[4w+k]).
// The integer model of this datapath is verilog/dv/audit/ref_models.py::nn_infer.

`default_nettype none

module nn_engine (
    input  wire        clk,
    input  wire        rst,

    // Input interface (features from feature extraction)
    input  wire        start,
    input  wire [7:0]  feature_in,     // Feature data read port (unsigned byte)
    output reg  [2:0]  feature_addr,   // Feature address (0-7)

    // Output interface
    output reg         done,
    output reg  [1:0]  class_id,       // Classification result (0-3)
    output reg  [7:0]  confidence,     // Confidence score (winning accumulator, clamped)
    output reg         busy,

    // Weight write port (one byte per write)
    input  wire        wt_wr_en,
    input  wire [7:0]  wt_wr_addr,     // 0-211: weight address
    input  wire [7:0]  wt_wr_data,     // INT8 weight value

    // Weight read port (one word per read)
    input  wire [5:0]  wt_rd_waddr,    // word index 0-52
    output wire [31:0] wt_rd_wdata     // {weight[4w+3], weight[4w+2], weight[4w+1], weight[4w]}
);

    // --- Weight storage ---
    // [0..127]   : Layer 1 weights (row-major: w[neuron][input])
    // [128..143] : Layer 1 biases
    // [144..207] : Layer 2 weights (row-major: w[neuron][input])
    // [208..211] : Layer 2 biases
    reg signed [7:0] weights [0:211];

    integer wi;
    always @(posedge clk) begin
        if (rst) begin
            for (wi = 0; wi < 212; wi = wi + 1)
                weights[wi] <= 8'sd0;
        end else if (wt_wr_en && wt_wr_addr < 8'd212) begin
            weights[wt_wr_addr] <= wt_wr_data;
        end
    end

    // --- Weight read port ---
    wire [7:0] rd_b0 = {wt_rd_waddr, 2'b00};
    assign wt_rd_wdata = (wt_rd_waddr > 6'd52) ? 32'd0 :
                         {weights[rd_b0 + 8'd3], weights[rd_b0 + 8'd2],
                          weights[rd_b0 + 8'd1], weights[rd_b0]};

    // --- Activations ---
    reg        [7:0]  inputs     [0:7];   // unsigned feature bytes
    reg signed [15:0] hidden     [0:15];  // layer-1 accumulators, wrapped to 16 bits, then ReLU
    reg signed [23:0] output_act [0:3];   // layer-2 accumulators, full 24-bit precision

    // --- FSM ---
    localparam S_IDLE     = 3'd0;
    localparam S_LOAD_IN  = 3'd1;
    localparam S_LAYER1   = 3'd2;
    localparam S_RELU     = 3'd3;
    localparam S_LAYER2   = 3'd4;
    localparam S_ARGMAX   = 3'd5;
    localparam S_DONE     = 3'd6;

    reg [2:0]  state;
    reg [3:0]  neuron_idx;     // Current neuron (0-15 for L1, 0-3 for L2)
    reg [3:0]  input_idx;      // Current input index
    reg signed [23:0] acc;     // Accumulator for MAC
    reg [3:0]  load_cnt;       // Input loading counter

    // Argmax
    reg signed [23:0] max_val;
    reg [1:0]  max_idx;
    reg [1:0]  argmax_cnt;
    reg        argmax_done;

    // --- MAC operands for the current step (all signed, all extended to 24 bits) ---
    wire [7:0]         w1_addr = {neuron_idx[3:0], input_idx[2:0]};                 // 0..127
    wire [7:0]         w2_addr = 8'd144 + {2'b00, neuron_idx[1:0], input_idx[3:0]}; // 144..207
    wire signed [8:0]  x_s     = {1'b0, inputs[input_idx[2:0]]};                    // unsigned byte -> signed 9 bit
    wire signed [16:0] p1      = x_s * weights[w1_addr];                            // 9 x 8 signed
    wire signed [23:0] p1x     = {{7{p1[16]}}, p1};
    wire signed [23:0] p2      = hidden[input_idx] * weights[w2_addr];              // 16 x 8 signed, 24-bit context
    wire signed [23:0] b1x     = {{16{weights[128 + neuron_idx][7]}}, weights[128 + neuron_idx]};
    wire signed [23:0] b2x     = {{16{weights[208 + neuron_idx[1:0]][7]}}, weights[208 + neuron_idx[1:0]]};

    integer i;
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
            load_cnt     <= 4'd0;
            acc          <= 24'sd0;
            argmax_done  <= 1'b0;
            argmax_cnt   <= 2'd0;
            max_val      <= 24'sd0;
            max_idx      <= 2'd0;
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
                // feature_in reflects the address registered on the previous edge.
                S_LOAD_IN: begin
                    inputs[load_cnt[2:0]] <= feature_in;
                    if (load_cnt == 4'd7) begin
                        state      <= S_LAYER1;
                        neuron_idx <= 4'd0;
                        input_idx  <= 4'd0;
                        acc        <= 24'sd0;
                    end else begin
                        load_cnt     <= load_cnt + 4'd1;
                        feature_addr <= load_cnt[2:0] + 3'd1;
                    end
                end

                // --- Layer 1: 8 -> 16, one product per clock ---
                S_LAYER1: begin
                    acc <= acc + p1x;
                    if (input_idx == 4'd7) begin
                        // last product and bias; the assignment wraps the sum to 16 bits
                        hidden[neuron_idx] <= acc + p1x + b1x;
                        if (neuron_idx == 4'd15) begin
                            state      <= S_RELU;
                            neuron_idx <= 4'd0;
                        end else begin
                            neuron_idx <= neuron_idx + 4'd1;
                            input_idx  <= 4'd0;
                            acc        <= 24'sd0;
                        end
                    end else begin
                        input_idx <= input_idx + 4'd1;
                    end
                end

                // --- ReLU on all hidden registers in one cycle ---
                S_RELU: begin
                    for (i = 0; i < 16; i = i + 1)
                        if (hidden[i][15]) hidden[i] <= 16'sd0;
                    state      <= S_LAYER2;
                    neuron_idx <= 4'd0;
                    input_idx  <= 4'd0;
                    acc        <= 24'sd0;
                end

                // --- Layer 2: 16 -> 4, one product per clock ---
                S_LAYER2: begin
                    acc <= acc + p2;
                    if (input_idx == 4'd15) begin
                        output_act[neuron_idx[1:0]] <= acc + p2 + b2x;
                        if (neuron_idx[1:0] == 2'd3) begin
                            state       <= S_ARGMAX;
                            argmax_cnt  <= 2'd0;
                            argmax_done <= 1'b0;
                            max_val     <= 24'sh800000;  // most negative
                            max_idx     <= 2'd0;
                        end else begin
                            neuron_idx <= neuron_idx + 4'd1;
                            input_idx  <= 4'd0;
                            acc        <= 24'sd0;
                        end
                    end else begin
                        input_idx <= input_idx + 4'd1;
                    end
                end

                // --- Argmax: first index of the maximum ---
                S_ARGMAX: begin
                    if (!argmax_done) begin
                        if (output_act[argmax_cnt] > max_val) begin
                            max_val <= output_act[argmax_cnt];
                            max_idx <= argmax_cnt;
                        end
                        if (argmax_cnt == 2'd3) argmax_done <= 1'b1;
                        else                    argmax_cnt  <= argmax_cnt + 2'd1;
                    end else begin
                        class_id   <= max_idx;
                        // confidence: winning accumulator clamped to [0, 255]
                        confidence <= (max_val[23]) ? 8'd0 :
                                      (max_val[23:8] != 16'd0) ? 8'hFF : max_val[7:0];
                        state      <= S_DONE;
                    end
                end

                S_DONE: begin
                    done  <= 1'b1;
                    busy  <= 1'b0;
                    state <= S_IDLE;
                end

                default: state <= S_IDLE;
            endcase
        end
    end

endmodule

`default_nettype wire

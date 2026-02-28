// SPDX-License-Identifier: Apache-2.0
// SenseEdge - 64-Point Radix-2 DIT FFT Engine
// Fixed-point: 16-bit input, 24-bit internal, 16-bit magnitude output
// Single butterfly unit, time-multiplexed across all stages
// Outputs 32 magnitude bins (DC to Nyquist)

`default_nettype none

module fft_engine (
    input  wire        clk,
    input  wire        rst,

    // Input interface
    input  wire        start,          // Pulse to begin FFT computation
    input  wire [15:0] sample_in,      // Sample data from buffer
    output reg  [5:0]  sample_addr,    // Address to read from sample buffer

    // Output interface
    output reg         done,           // Pulses when FFT complete
    output wire [15:0] mag_out,        // Magnitude bin read port
    input  wire [4:0]  mag_addr,       // Magnitude bin address (0-31)
    output reg         busy
);

    // --- Internal storage ---
    // Real and imaginary parts, 24-bit to preserve precision
    reg signed [23:0] data_re [0:63];
    reg signed [23:0] data_im [0:63];

    // Magnitude output buffer (16-bit unsigned)
    reg [15:0] mag_buf [0:31];

    assign mag_out = mag_buf[mag_addr];

    // --- Twiddle factor ROM ---
    // W(k,N) = cos(2*pi*k/N) - j*sin(2*pi*k/N)
    // Stored as Q1.14 fixed-point (scaled by 16384)
    // Only need 32 twiddle factors for 64-point FFT
    reg signed [15:0] twiddle_re [0:31];
    reg signed [15:0] twiddle_im [0:31];

    // Initialize twiddle factors (cos and -sin values, Q1.14)
    integer tw_i;
    initial begin
        // W(k,64) = cos(2*pi*k/64) - j*sin(2*pi*k/64), scaled by 16384
        twiddle_re[ 0] =  16384; twiddle_im[ 0] =      0;
        twiddle_re[ 1] =  16305; twiddle_im[ 1] =  -1608;
        twiddle_re[ 2] =  16069; twiddle_im[ 2] =  -3196;
        twiddle_re[ 3] =  15679; twiddle_im[ 3] =  -4756;
        twiddle_re[ 4] =  15137; twiddle_im[ 4] =  -6270;
        twiddle_re[ 5] =  14449; twiddle_im[ 5] =  -7723;
        twiddle_re[ 6] =  13623; twiddle_im[ 6] =  -9102;
        twiddle_re[ 7] =  12665; twiddle_im[ 7] = -10394;
        twiddle_re[ 8] =  11585; twiddle_im[ 8] = -11585;
        twiddle_re[ 9] =  10394; twiddle_im[ 9] = -12665;
        twiddle_re[10] =   9102; twiddle_im[10] = -13623;
        twiddle_re[11] =   7723; twiddle_im[11] = -14449;
        twiddle_re[12] =   6270; twiddle_im[12] = -15137;
        twiddle_re[13] =   4756; twiddle_im[13] = -15679;
        twiddle_re[14] =   3196; twiddle_im[14] = -16069;
        twiddle_re[15] =   1608; twiddle_im[15] = -16305;
        twiddle_re[16] =      0; twiddle_im[16] = -16384;
        twiddle_re[17] =  -1608; twiddle_im[17] = -16305;
        twiddle_re[18] =  -3196; twiddle_im[18] = -16069;
        twiddle_re[19] =  -4756; twiddle_im[19] = -15679;
        twiddle_re[20] =  -6270; twiddle_im[20] = -15137;
        twiddle_re[21] =  -7723; twiddle_im[21] = -14449;
        twiddle_re[22] =  -9102; twiddle_im[22] = -13623;
        twiddle_re[23] = -10394; twiddle_im[23] = -12665;
        twiddle_re[24] = -11585; twiddle_im[24] = -11585;
        twiddle_re[25] = -12665; twiddle_im[25] = -10394;
        twiddle_re[26] = -13623; twiddle_im[26] =  -9102;
        twiddle_re[27] = -14449; twiddle_im[27] =  -7723;
        twiddle_re[28] = -15137; twiddle_im[28] =  -6270;
        twiddle_re[29] = -15679; twiddle_im[29] =  -4756;
        twiddle_re[30] = -16069; twiddle_im[30] =  -3196;
        twiddle_re[31] = -16305; twiddle_im[31] =  -1608;
    end

    // --- Bit-reversal table for 64-point ---
    function [5:0] bit_reverse;
        input [5:0] idx;
        begin
            bit_reverse = {idx[0], idx[1], idx[2], idx[3], idx[4], idx[5]};
        end
    endfunction

    // --- FSM States ---
    localparam S_IDLE       = 3'd0;
    localparam S_LOAD       = 3'd1;
    localparam S_BUTTERFLY  = 3'd2;
    localparam S_BF_COMPUTE = 3'd3;
    localparam S_MAGNITUDE  = 3'd4;
    localparam S_DONE       = 3'd5;

    reg [2:0]  state;
    reg [5:0]  load_cnt;        // Loading counter
    reg [2:0]  stage;           // FFT stage (0-5 for 64-point)
    reg [4:0]  bf_idx;          // Butterfly index within group
    reg [4:0]  grp_idx;         // Group index within stage
    reg [5:0]  half_size;       // Half the group size for current stage
    reg [4:0]  num_groups;      // Number of groups in current stage
    reg [4:0]  mag_cnt;         // Magnitude computation counter

    // Butterfly computation intermediates
    reg signed [23:0] ar, ai, br, bi;
    reg signed [15:0] wr, wi;
    reg signed [39:0] prod_re, prod_im;    // 24 * 16 = 40 bits
    reg signed [23:0] tr, ti;
    reg [5:0] idx_p, idx_q;
    reg [4:0] tw_idx;

    always @(posedge clk) begin
        if (rst) begin
            state       <= S_IDLE;
            done        <= 1'b0;
            busy        <= 1'b0;
            sample_addr <= 6'd0;
            load_cnt    <= 6'd0;
            stage       <= 3'd0;
            bf_idx      <= 5'd0;
            grp_idx     <= 5'd0;
            mag_cnt     <= 5'd0;
        end else begin
            done <= 1'b0;

            case (state)
                // --- Wait for start signal ---
                S_IDLE: begin
                    if (start) begin
                        state       <= S_LOAD;
                        busy        <= 1'b1;
                        load_cnt    <= 6'd0;
                        sample_addr <= bit_reverse(6'd0);
                    end
                end

                // --- Load samples with bit-reversal ---
                S_LOAD: begin
                    data_re[load_cnt] <= {{8{sample_in[15]}}, sample_in};  // Sign-extend to 24-bit
                    data_im[load_cnt] <= 24'd0;

                    if (load_cnt == 6'd63) begin
                        state     <= S_BUTTERFLY;
                        stage     <= 3'd0;
                        grp_idx   <= 5'd0;
                        bf_idx    <= 5'd0;
                        half_size <= 6'd1;    // Stage 0: groups of 2
                        num_groups <= 5'd31;  // 32 groups for stage 0
                    end else begin
                        load_cnt    <= load_cnt + 6'd1;
                        sample_addr <= bit_reverse(load_cnt + 6'd1);
                    end
                end

                // --- Setup butterfly indices ---
                S_BUTTERFLY: begin
                    // Compute indices for this butterfly
                    idx_p  <= {grp_idx, bf_idx[0], bf_idx[4:1]} & 6'h3F;  // Will be overridden below
                    idx_q  <= ({grp_idx, bf_idx[0], bf_idx[4:1]} & 6'h3F) + half_size;

                    // Proper index computation per stage
                    case (stage)
                        3'd0: begin
                            idx_p <= {bf_idx, 1'b0};
                            idx_q <= {bf_idx, 1'b1};
                            tw_idx <= 5'd0;
                        end
                        3'd1: begin
                            idx_p <= {grp_idx[4:0], 2'b00} + {4'd0, bf_idx[0]};
                            idx_q <= {grp_idx[4:0], 2'b00} + {4'd0, bf_idx[0]} + 6'd2;
                            tw_idx <= {bf_idx[0], 4'd0};
                        end
                        3'd2: begin
                            idx_p <= {grp_idx[4:0], 3'b000} + {3'd0, bf_idx[1:0]};
                            idx_q <= {grp_idx[4:0], 3'b000} + {3'd0, bf_idx[1:0]} + 6'd4;
                            tw_idx <= {bf_idx[1:0], 3'd0};
                        end
                        3'd3: begin
                            idx_p <= {grp_idx[4:0], 4'b0000} + {2'd0, bf_idx[2:0]};
                            idx_q <= {grp_idx[4:0], 4'b0000} + {2'd0, bf_idx[2:0]} + 6'd8;
                            tw_idx <= {bf_idx[2:0], 2'd0};
                        end
                        3'd4: begin
                            idx_p <= {grp_idx[4:0], 5'b00000} + {1'd0, bf_idx[3:0]};
                            idx_q <= {grp_idx[4:0], 5'b00000} + {1'd0, bf_idx[3:0]} + 6'd16;
                            tw_idx <= {bf_idx[3:0], 1'd0};
                        end
                        3'd5: begin
                            idx_p <= {1'b0, bf_idx[4:0]};
                            idx_q <= {1'b0, bf_idx[4:0]} + 6'd32;
                            tw_idx <= bf_idx[4:0];
                        end
                        default: begin
                            idx_p <= 6'd0;
                            idx_q <= 6'd0;
                            tw_idx <= 5'd0;
                        end
                    endcase

                    state <= S_BF_COMPUTE;
                end

                // --- Perform butterfly computation ---
                S_BF_COMPUTE: begin
                    // Read inputs
                    ar = data_re[idx_p];
                    ai = data_im[idx_p];
                    br = data_re[idx_q];
                    bi = data_im[idx_q];
                    wr = twiddle_re[tw_idx];
                    wi = twiddle_im[tw_idx];

                    // Complex multiply: (br + j*bi) * (wr + j*wi)
                    // Real part: br*wr - bi*wi
                    // Imag part: br*wi + bi*wr
                    prod_re = (br * wr) - (bi * wi);
                    prod_im = (br * wi) + (bi * wr);

                    // Scale back (divide by 16384 = right shift 14)
                    tr = prod_re[37:14];
                    ti = prod_im[37:14];

                    // Butterfly: p' = a + t, q' = a - t
                    data_re[idx_p] <= ar + tr;
                    data_im[idx_p] <= ai + ti;
                    data_re[idx_q] <= ar - tr;
                    data_im[idx_q] <= ai - ti;

                    // Advance to next butterfly
                    case (stage)
                        3'd0: begin
                            if (bf_idx == 5'd31) begin
                                stage     <= 3'd1;
                                bf_idx    <= 5'd0;
                                grp_idx   <= 5'd0;
                                half_size <= 6'd2;
                                num_groups <= 5'd15;
                            end else begin
                                bf_idx <= bf_idx + 5'd1;
                            end
                            state <= S_BUTTERFLY;
                        end
                        3'd1: begin
                            if (bf_idx[0] == 1'b1) begin
                                if (grp_idx == num_groups) begin
                                    stage     <= 3'd2;
                                    bf_idx    <= 5'd0;
                                    grp_idx   <= 5'd0;
                                    half_size <= 6'd4;
                                    num_groups <= 5'd7;
                                end else begin
                                    grp_idx <= grp_idx + 5'd1;
                                    bf_idx  <= 5'd0;
                                end
                            end else begin
                                bf_idx <= bf_idx + 5'd1;
                            end
                            state <= S_BUTTERFLY;
                        end
                        3'd2: begin
                            if (bf_idx[1:0] == 2'd3) begin
                                if (grp_idx == num_groups) begin
                                    stage     <= 3'd3;
                                    bf_idx    <= 5'd0;
                                    grp_idx   <= 5'd0;
                                    half_size <= 6'd8;
                                    num_groups <= 5'd3;
                                end else begin
                                    grp_idx <= grp_idx + 5'd1;
                                    bf_idx  <= 5'd0;
                                end
                            end else begin
                                bf_idx <= bf_idx + 5'd1;
                            end
                            state <= S_BUTTERFLY;
                        end
                        3'd3: begin
                            if (bf_idx[2:0] == 3'd7) begin
                                if (grp_idx == num_groups) begin
                                    stage     <= 3'd4;
                                    bf_idx    <= 5'd0;
                                    grp_idx   <= 5'd0;
                                    half_size <= 6'd16;
                                    num_groups <= 5'd1;
                                end else begin
                                    grp_idx <= grp_idx + 5'd1;
                                    bf_idx  <= 5'd0;
                                end
                            end else begin
                                bf_idx <= bf_idx + 5'd1;
                            end
                            state <= S_BUTTERFLY;
                        end
                        3'd4: begin
                            if (bf_idx[3:0] == 4'd15) begin
                                if (grp_idx == num_groups) begin
                                    stage     <= 3'd5;
                                    bf_idx    <= 5'd0;
                                    grp_idx   <= 5'd0;
                                    half_size <= 6'd32;
                                    num_groups <= 5'd0;
                                end else begin
                                    grp_idx <= grp_idx + 5'd1;
                                    bf_idx  <= 5'd0;
                                end
                            end else begin
                                bf_idx <= bf_idx + 5'd1;
                            end
                            state <= S_BUTTERFLY;
                        end
                        3'd5: begin
                            if (bf_idx == 5'd31) begin
                                // All stages complete, compute magnitudes
                                state   <= S_MAGNITUDE;
                                mag_cnt <= 5'd0;
                            end else begin
                                bf_idx <= bf_idx + 5'd1;
                                state  <= S_BUTTERFLY;
                            end
                        end
                        default: state <= S_IDLE;
                    endcase
                end

                // --- Compute magnitude approximation ---
                // |X| ≈ max(|Re|, |Im|) + 0.5 * min(|Re|, |Im|)
                S_MAGNITUDE: begin
                    begin : mag_block
                        reg [23:0] abs_re, abs_im, mag_max, mag_min;
                        abs_re  = data_re[mag_cnt][23] ? -data_re[mag_cnt] : data_re[mag_cnt];
                        abs_im  = data_im[mag_cnt][23] ? -data_im[mag_cnt] : data_im[mag_cnt];
                        mag_max = (abs_re > abs_im) ? abs_re : abs_im;
                        mag_min = (abs_re > abs_im) ? abs_im : abs_re;
                        // Saturate to 16-bit
                        mag_buf[mag_cnt] <= (mag_max[23:16] != 0) ? 16'hFFFF
                                          : mag_max[15:0] + {1'b0, mag_min[15:1]};
                    end

                    if (mag_cnt == 5'd31) begin
                        state <= S_DONE;
                    end else begin
                        mag_cnt <= mag_cnt + 5'd1;
                    end
                end

                // --- Signal completion ---
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

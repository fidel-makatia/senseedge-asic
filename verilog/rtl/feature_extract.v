// SPDX-License-Identifier: Apache-2.0
// SenseEdge - Feature Extraction Engine
// Computes 8 spectral features from 32 FFT magnitude bins
// Outputs normalized 8-bit features for neural network input
//
// Corrected RTL (branch fix/audit-defects).  Change against the fabricated revision:
//   The magnitude store is read combinationally (mag_in = mag_buf[mag_addr] in the same
//   cycle), but the fabricated extractor paired each value with the address of the
//   previous cycle, so accumulator j received |X_{(j+1) mod 32}| and bin 0 was never
//   accumulated.  The scan now accumulates mag_in under the address that produced it.
//   Bands are therefore bins 1-4, 5-10, 11-20, 21-31 as designed; the peak, centroid and
//   total run over bins 0-31.  The integer model is verilog/dv/audit/ref_models.py::features.
//   The scan is one cycle shorter than before (32 scan + normalise + done).

`default_nettype none

module feature_extract (
    input  wire        clk,
    input  wire        rst,

    // Input interface (FFT magnitude bins)
    input  wire        start,          // Pulse to begin feature extraction
    output reg  [4:0]  mag_addr,       // Address to read FFT magnitude bin
    input  wire [15:0] mag_in,         // Magnitude bin data (combinational read)

    // Output interface
    output reg         done,           // Pulses when features ready
    output wire [7:0]  feature_out,    // Feature read port
    input  wire [2:0]  feature_addr,   // Feature address (0-7)
    output reg         busy
);

    // --- Feature storage (8 features, 8-bit each) ---
    reg [7:0] features [0:7];
    assign feature_out = features[feature_addr];

    // Feature indices:
    // [0] Band energy low     (bins 1-4)
    // [1] Band energy mid-low (bins 5-10)
    // [2] Band energy mid-hi  (bins 11-20)
    // [3] Band energy high    (bins 21-31)
    // [4] Peak frequency      (bin index of max magnitude, times 8)
    // [5] Peak magnitude      (value at peak bin, top 8 bits)
    // [6] Spectral centroid   (bits 23:16 of sum(bin * mag))
    // [7] Total energy        (sum of all bins, saturating bit selection)

    // --- FSM ---
    localparam S_IDLE    = 3'd0;
    localparam S_SCAN    = 3'd1;
    localparam S_NORM    = 3'd3;
    localparam S_DONE    = 3'd4;

    reg [2:0]  state;
    reg [4:0]  scan_idx;        // == mag_addr during S_SCAN

    // Accumulators
    reg [23:0] band_low;        // Sum bins 1-4
    reg [23:0] band_midlow;     // Sum bins 5-10
    reg [23:0] band_midhi;      // Sum bins 11-20
    reg [23:0] band_high;       // Sum bins 21-31
    reg [15:0] peak_mag;        // Maximum magnitude seen
    reg [4:0]  peak_bin;        // Bin index of maximum (first occurrence)
    reg [31:0] weighted_sum;    // sum(bin * mag) for the centroid
    reg [23:0] total_energy;    // Sum of all bins

    always @(posedge clk) begin
        if (rst) begin
            state       <= S_IDLE;
            done        <= 1'b0;
            busy        <= 1'b0;
            mag_addr    <= 5'd0;
            scan_idx    <= 5'd0;
        end else begin
            done <= 1'b0;

            case (state)
                S_IDLE: begin
                    if (start) begin
                        state        <= S_SCAN;
                        busy         <= 1'b1;
                        scan_idx     <= 5'd0;
                        mag_addr     <= 5'd0;
                        band_low     <= 24'd0;
                        band_midlow  <= 24'd0;
                        band_midhi   <= 24'd0;
                        band_high    <= 24'd0;
                        peak_mag     <= 16'd0;
                        peak_bin     <= 5'd0;
                        weighted_sum <= 32'd0;
                        total_energy <= 24'd0;
                    end
                end

                // One bin per cycle: mag_in is mag_buf[mag_addr] and mag_addr == scan_idx.
                S_SCAN: begin
                    total_energy <= total_energy + {8'd0, mag_in};
                    weighted_sum <= weighted_sum + ({16'd0, mag_in} * {27'd0, scan_idx});
                    if (mag_in > peak_mag) begin
                        peak_mag <= mag_in;
                        peak_bin <= scan_idx;
                    end
                    if (scan_idx >= 5'd1  && scan_idx <= 5'd4)  band_low    <= band_low    + {8'd0, mag_in};
                    if (scan_idx >= 5'd5  && scan_idx <= 5'd10) band_midlow <= band_midlow + {8'd0, mag_in};
                    if (scan_idx >= 5'd11 && scan_idx <= 5'd20) band_midhi  <= band_midhi  + {8'd0, mag_in};
                    if (scan_idx >= 5'd21)                       band_high   <= band_high   + {8'd0, mag_in};

                    if (scan_idx == 5'd31) begin
                        state    <= S_NORM;
                        mag_addr <= 5'd0;
                    end else begin
                        scan_idx <= scan_idx + 5'd1;
                        mag_addr <= scan_idx + 5'd1;
                    end
                end

                // Normalize features to 8 bits
                S_NORM: begin
                    features[0] <= (band_low[23:16]    != 8'd0) ? 8'hFF : band_low[15:8];
                    features[1] <= (band_midlow[23:16] != 8'd0) ? 8'hFF : band_midlow[15:8];
                    features[2] <= (band_midhi[23:16]  != 8'd0) ? 8'hFF : band_midhi[15:8];
                    features[3] <= (band_high[23:16]   != 8'd0) ? 8'hFF : band_high[15:8];
                    features[4] <= {peak_bin, 3'b000};
                    features[5] <= peak_mag[15:8];
                    features[6] <= (total_energy == 24'd0) ? 8'd0 : weighted_sum[23:16];
                    features[7] <= (total_energy[23:16] != 8'd0) ? 8'hFF : total_energy[15:8];
                    state <= S_DONE;
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

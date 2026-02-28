// SPDX-License-Identifier: Apache-2.0
// SenseEdge - Feature Extraction Engine
// Computes 8 spectral features from 32 FFT magnitude bins
// Outputs normalized 8-bit features for neural network input

`default_nettype none

module feature_extract (
    input  wire        clk,
    input  wire        rst,

    // Input interface (FFT magnitude bins)
    input  wire        start,          // Pulse to begin feature extraction
    output reg  [4:0]  mag_addr,       // Address to read FFT magnitude bin
    input  wire [15:0] mag_in,         // Magnitude bin data

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
    // [4] Peak frequency      (bin index of max magnitude)
    // [5] Peak magnitude      (value at peak bin, scaled)
    // [6] Spectral centroid   (weighted average frequency)
    // [7] Total energy        (sum of all bins, scaled)

    // --- FSM ---
    localparam S_IDLE    = 3'd0;
    localparam S_SCAN    = 3'd1;
    localparam S_ACCUM   = 3'd2;
    localparam S_NORM    = 3'd3;
    localparam S_DONE    = 3'd4;

    reg [2:0]  state;
    reg [4:0]  scan_idx;

    // Accumulators
    reg [23:0] band_low;        // Sum bins 1-4
    reg [23:0] band_midlow;     // Sum bins 5-10
    reg [23:0] band_midhi;      // Sum bins 11-20
    reg [23:0] band_high;       // Sum bins 21-31
    reg [15:0] peak_mag;        // Maximum magnitude seen
    reg [4:0]  peak_bin;        // Bin index of maximum
    reg [31:0] weighted_sum;    // For spectral centroid: sum(bin * mag)
    reg [23:0] total_energy;    // Sum of all bins

    // Pipeline delay for memory read
    reg [4:0]  scan_idx_d;
    reg        accum_valid;

    always @(posedge clk) begin
        if (rst) begin
            state       <= S_IDLE;
            done        <= 1'b0;
            busy        <= 1'b0;
            mag_addr    <= 5'd0;
            scan_idx    <= 5'd0;
            accum_valid <= 1'b0;
        end else begin
            done        <= 1'b0;
            accum_valid <= 1'b0;

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

                // Read each bin sequentially, pipeline by 1 cycle
                S_SCAN: begin
                    scan_idx_d  <= scan_idx;
                    accum_valid <= (scan_idx > 5'd0);  // First read has 1-cycle latency

                    if (scan_idx == 5'd31) begin
                        state    <= S_ACCUM;
                        mag_addr <= 5'd0;
                    end else begin
                        scan_idx <= scan_idx + 5'd1;
                        mag_addr <= scan_idx + 5'd1;
                    end
                end

                // Process last bin
                S_ACCUM: begin
                    accum_valid <= 1'b1;
                    scan_idx_d  <= 5'd31;
                    state       <= S_NORM;
                end

                // Normalize features to 8-bit
                S_NORM: begin
                    // Band energies: right-shift to fit 8 bits
                    features[0] <= (band_low[23:16]    != 0) ? 8'hFF : band_low[15:8];
                    features[1] <= (band_midlow[23:16] != 0) ? 8'hFF : band_midlow[15:8];
                    features[2] <= (band_midhi[23:16]  != 0) ? 8'hFF : band_midhi[15:8];
                    features[3] <= (band_high[23:16]   != 0) ? 8'hFF : band_high[15:8];

                    // Peak bin index scaled to 0-255 range: (peak_bin * 255) / 31 ≈ peak_bin * 8
                    features[4] <= {peak_bin, 3'b000};

                    // Peak magnitude (top 8 bits)
                    features[5] <= peak_mag[15:8];

                    // Spectral centroid: weighted_sum / total_energy, scaled
                    // Approximate: use top bits of weighted_sum
                    features[6] <= (total_energy == 0) ? 8'd0 : weighted_sum[23:16];

                    // Total energy (top 8 bits)
                    features[7] <= (total_energy[23:16] != 0) ? 8'hFF : total_energy[15:8];

                    state <= S_DONE;
                end

                S_DONE: begin
                    done <= 1'b1;
                    busy <= 1'b0;
                    state <= S_IDLE;
                end

                default: state <= S_IDLE;
            endcase

            // --- Accumulation logic (runs during S_SCAN and S_ACCUM) ---
            if (accum_valid) begin
                // Total energy
                total_energy <= total_energy + {8'd0, mag_in};

                // Weighted sum for centroid
                weighted_sum <= weighted_sum + (mag_in * scan_idx_d);

                // Peak detection
                if (mag_in > peak_mag) begin
                    peak_mag <= mag_in;
                    peak_bin <= scan_idx_d;
                end

                // Band accumulation
                if (scan_idx_d >= 5'd1 && scan_idx_d <= 5'd4)
                    band_low <= band_low + {8'd0, mag_in};
                if (scan_idx_d >= 5'd5 && scan_idx_d <= 5'd10)
                    band_midlow <= band_midlow + {8'd0, mag_in};
                if (scan_idx_d >= 5'd11 && scan_idx_d <= 5'd20)
                    band_midhi <= band_midhi + {8'd0, mag_in};
                if (scan_idx_d >= 5'd21 && scan_idx_d <= 5'd31)
                    band_high <= band_high + {8'd0, mag_in};
            end
        end
    end

endmodule

`default_nettype wire

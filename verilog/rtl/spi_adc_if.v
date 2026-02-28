// SPDX-License-Identifier: Apache-2.0
// SenseEdge - SPI ADC Interface
// Configurable SPI master for external ADC (e.g., MCP3201, ADS7042)
// Collects 64 samples into a circular buffer, triggers FFT on completion

`default_nettype none

module spi_adc_if (
    input  wire        clk,
    input  wire        rst,

    // Control
    input  wire        enable,
    input  wire [15:0] clk_div,       // SPI clock divider (sample rate control)

    // SPI pins
    output reg         spi_clk,
    output reg         spi_cs_n,
    input  wire        spi_miso,

    // Sample buffer output (to FFT)
    output reg         samples_valid,  // Pulses when 64 samples collected
    output wire [15:0] sample_out,     // Sample data read port
    input  wire [5:0]  sample_addr,    // Sample address (0-63)

    // Status
    output wire [5:0]  sample_count
);

    // --- Parameters ---
    localparam SAMPLE_DEPTH = 64;
    localparam ADC_BITS     = 12;

    // --- State Machine ---
    localparam S_IDLE    = 3'd0;
    localparam S_CS_LOW  = 3'd1;
    localparam S_SHIFT   = 3'd2;
    localparam S_CS_HIGH = 3'd3;
    localparam S_WAIT    = 3'd4;

    reg [2:0]  state;
    reg [15:0] clk_cnt;         // Clock divider counter
    reg        spi_clk_en;      // SPI clock edge trigger
    reg [4:0]  bit_cnt;         // Bits shifted in (0-15)
    reg [15:0] shift_reg;       // SPI shift register
    reg [5:0]  wr_ptr;          // Write pointer into sample buffer
    reg [15:0] sample_buf [0:SAMPLE_DEPTH-1];

    assign sample_out   = sample_buf[sample_addr];
    assign sample_count = wr_ptr;

    // --- SPI Clock Divider ---
    always @(posedge clk) begin
        if (rst || !enable) begin
            clk_cnt    <= 16'd0;
            spi_clk_en <= 1'b0;
        end else begin
            if (clk_cnt >= clk_div) begin
                clk_cnt    <= 16'd0;
                spi_clk_en <= 1'b1;
            end else begin
                clk_cnt    <= clk_cnt + 16'd1;
                spi_clk_en <= 1'b0;
            end
        end
    end

    // --- SPI Master State Machine ---
    always @(posedge clk) begin
        if (rst) begin
            state         <= S_IDLE;
            spi_clk       <= 1'b0;
            spi_cs_n      <= 1'b1;
            bit_cnt       <= 5'd0;
            shift_reg     <= 16'd0;
            wr_ptr        <= 6'd0;
            samples_valid <= 1'b0;
        end else begin
            samples_valid <= 1'b0;  // Default: single-cycle pulse

            case (state)
                S_IDLE: begin
                    spi_cs_n <= 1'b1;
                    spi_clk  <= 1'b0;
                    if (enable && spi_clk_en) begin
                        state <= S_CS_LOW;
                    end
                end

                S_CS_LOW: begin
                    spi_cs_n  <= 1'b0;
                    bit_cnt   <= 5'd0;
                    shift_reg <= 16'd0;
                    if (spi_clk_en) begin
                        state <= S_SHIFT;
                    end
                end

                S_SHIFT: begin
                    if (spi_clk_en) begin
                        spi_clk <= ~spi_clk;
                        if (spi_clk) begin
                            // Sample on falling edge
                            shift_reg <= {shift_reg[14:0], spi_miso};
                            bit_cnt   <= bit_cnt + 5'd1;
                            if (bit_cnt == 5'd15) begin
                                state <= S_CS_HIGH;
                            end
                        end
                    end
                end

                S_CS_HIGH: begin
                    spi_cs_n <= 1'b1;
                    spi_clk  <= 1'b0;
                    // Store sample (sign-extend 12-bit to 16-bit signed)
                    sample_buf[wr_ptr] <= {{(16-ADC_BITS){shift_reg[ADC_BITS-1]}}, shift_reg[ADC_BITS-1:0]};
                    wr_ptr <= wr_ptr + 6'd1;
                    if (wr_ptr == 6'd63) begin
                        samples_valid <= 1'b1;
                    end
                    state <= S_WAIT;
                end

                S_WAIT: begin
                    // Inter-sample delay
                    if (spi_clk_en) begin
                        state <= S_IDLE;
                    end
                end

                default: state <= S_IDLE;
            endcase
        end
    end

endmodule

`default_nettype wire

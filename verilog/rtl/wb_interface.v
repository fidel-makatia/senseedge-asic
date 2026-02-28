// SPDX-License-Identifier: Apache-2.0
// SenseEdge - Wishbone Slave Interface & Control Registers
// 32-bit Wishbone B4 compliant slave
// Provides register access for control, status, FFT data, features, and NN weights

`default_nettype none

module wb_interface (
    input  wire        clk,
    input  wire        rst,

    // Wishbone slave port
    input  wire        wb_cyc_i,
    input  wire        wb_stb_i,
    input  wire        wb_we_i,
    input  wire [3:0]  wb_sel_i,
    input  wire [31:0] wb_adr_i,
    input  wire [31:0] wb_dat_i,
    output reg         wb_ack_o,
    output reg  [31:0] wb_dat_o,

    // Control outputs
    output reg         enable,
    output reg  [15:0] clk_div,
    output reg  [7:0]  alarm_threshold,
    output reg  [3:0]  fault_count_cfg, // Consecutive faults before alarm

    // Status inputs
    input  wire [1:0]  class_id,
    input  wire [7:0]  confidence,
    input  wire        fft_busy,
    input  wire        nn_busy,
    input  wire        fe_busy,
    input  wire        alarm_active,

    // FFT magnitude readback
    output reg  [4:0]  fft_rd_addr,
    input  wire [15:0] fft_rd_data,

    // Feature readback
    output reg  [2:0]  feature_rd_addr,
    input  wire [7:0]  feature_rd_data,

    // NN weight loading
    output reg         wt_wr_en,
    output reg  [7:0]  wt_wr_addr,
    output reg  [7:0]  wt_wr_data,

    // Interrupt
    input  wire        classification_done,
    input  wire        alarm_irq_in,
    output reg  [2:0]  irq
);

    // --- Address map (relative to base) ---
    // We use bits [7:0] of the address for register selection
    localparam ADDR_CTRL         = 8'h00;
    localparam ADDR_STATUS       = 8'h04;
    localparam ADDR_CLASS_RESULT = 8'h08;
    localparam ADDR_ALARM_CFG   = 8'h0C;
    localparam ADDR_FFT_DATA    = 8'h10;
    localparam ADDR_FEATURE_DATA = 8'h14;
    localparam ADDR_IRQ_FLAGS   = 8'h18;
    localparam ADDR_CLK_DIV     = 8'h1C;
    // 8'h20 - 8'h7F: NN weights (212 bytes, 53 x 32-bit words)
    localparam ADDR_NN_WEIGHTS_BASE = 8'h20;
    localparam ADDR_NN_WEIGHTS_END  = 8'h74; // 0x20 + 53*4 - 4

    // --- Internal registers ---
    reg [2:0]  irq_flags;       // [0] classification done, [1] alarm, [2] reserved
    reg [2:0]  irq_enable;
    reg [4:0]  fft_auto_addr;   // Auto-incrementing FFT read address
    reg [2:0]  feat_auto_addr;  // Auto-incrementing feature read address

    wire       wb_valid = wb_cyc_i && wb_stb_i;
    wire [7:0] reg_addr = wb_adr_i[7:0];

    // --- IRQ flag capture ---
    always @(posedge clk) begin
        if (rst) begin
            irq_flags <= 3'd0;
        end else begin
            // Set on event
            if (classification_done) irq_flags[0] <= 1'b1;
            if (alarm_irq_in)       irq_flags[1] <= 1'b1;

            // Clear on write to IRQ_FLAGS register
            if (wb_valid && wb_we_i && reg_addr == ADDR_IRQ_FLAGS) begin
                irq_flags <= irq_flags & ~wb_dat_i[2:0];
            end
        end
    end

    // IRQ output
    always @(*) begin
        irq[0] = |(irq_flags & irq_enable);
        irq[1] = 1'b0;
        irq[2] = 1'b0;
    end

    // --- Wishbone transaction handling ---
    always @(posedge clk) begin
        if (rst) begin
            wb_ack_o       <= 1'b0;
            wb_dat_o       <= 32'd0;
            enable         <= 1'b0;
            clk_div        <= 16'd249;  // Default: divide by 250
            alarm_threshold <= 8'd128;
            fault_count_cfg <= 4'd3;
            irq_enable     <= 3'd0;
            fft_auto_addr  <= 5'd0;
            feat_auto_addr <= 3'd0;
            wt_wr_en       <= 1'b0;
            fft_rd_addr    <= 5'd0;
            feature_rd_addr <= 3'd0;
        end else begin
            wb_ack_o <= 1'b0;
            wt_wr_en <= 1'b0;

            if (wb_valid && !wb_ack_o) begin
                wb_ack_o <= 1'b1;

                if (wb_we_i) begin
                    // --- Write operations ---
                    case (reg_addr)
                        ADDR_CTRL: begin
                            if (wb_sel_i[0]) enable    <= wb_dat_i[0];
                            if (wb_sel_i[1]) irq_enable <= wb_dat_i[10:8];
                        end
                        ADDR_ALARM_CFG: begin
                            if (wb_sel_i[0]) alarm_threshold <= wb_dat_i[7:0];
                            if (wb_sel_i[1]) fault_count_cfg <= wb_dat_i[11:8];
                        end
                        ADDR_CLK_DIV: begin
                            if (wb_sel_i[0]) clk_div[7:0]  <= wb_dat_i[7:0];
                            if (wb_sel_i[1]) clk_div[15:8] <= wb_dat_i[15:8];
                        end
                        ADDR_FFT_DATA: begin
                            // Write sets the auto-increment address
                            fft_auto_addr <= wb_dat_i[4:0];
                            fft_rd_addr   <= wb_dat_i[4:0];
                        end
                        ADDR_FEATURE_DATA: begin
                            feat_auto_addr  <= wb_dat_i[2:0];
                            feature_rd_addr <= wb_dat_i[2:0];
                        end
                        default: begin
                            // NN weight writes
                            if (reg_addr >= ADDR_NN_WEIGHTS_BASE && reg_addr <= ADDR_NN_WEIGHTS_END) begin
                                // Each 32-bit word writes up to 4 weight bytes
                                // Address mapping: (reg_addr - 0x20) gives word offset
                                // Byte address = word_offset * 4 + byte_lane
                                begin : wt_write_block
                                    reg [7:0] base_addr;
                                    base_addr = (reg_addr - ADDR_NN_WEIGHTS_BASE);  // Word offset * 4 (already aligned)

                                    // Write byte 0
                                    if (wb_sel_i[0] && base_addr < 8'd212) begin
                                        wt_wr_en   <= 1'b1;
                                        wt_wr_addr <= base_addr;
                                        wt_wr_data <= wb_dat_i[7:0];
                                    end
                                end
                            end
                        end
                    endcase
                end else begin
                    // --- Read operations ---
                    case (reg_addr)
                        ADDR_CTRL: begin
                            wb_dat_o <= {21'd0, irq_enable, 7'd0, enable};
                        end
                        ADDR_STATUS: begin
                            wb_dat_o <= {27'd0, alarm_active, fe_busy, nn_busy, fft_busy, enable};
                        end
                        ADDR_CLASS_RESULT: begin
                            wb_dat_o <= {22'd0, confidence, class_id};
                        end
                        ADDR_ALARM_CFG: begin
                            wb_dat_o <= {20'd0, fault_count_cfg, alarm_threshold};
                        end
                        ADDR_FFT_DATA: begin
                            wb_dat_o      <= {16'd0, fft_rd_data};
                            fft_auto_addr <= fft_auto_addr + 5'd1;
                            fft_rd_addr   <= fft_auto_addr + 5'd1;
                        end
                        ADDR_FEATURE_DATA: begin
                            wb_dat_o       <= {24'd0, feature_rd_data};
                            feat_auto_addr <= feat_auto_addr + 3'd1;
                            feature_rd_addr <= feat_auto_addr + 3'd1;
                        end
                        ADDR_IRQ_FLAGS: begin
                            wb_dat_o <= {29'd0, irq_flags};
                        end
                        ADDR_CLK_DIV: begin
                            wb_dat_o <= {16'd0, clk_div};
                        end
                        default: begin
                            wb_dat_o <= 32'd0;
                        end
                    endcase
                end
            end
        end
    end

endmodule

`default_nettype wire

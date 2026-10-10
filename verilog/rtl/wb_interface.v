// SPDX-License-Identifier: Apache-2.0
// SenseEdge - Wishbone Slave Interface & Control Registers
// 32-bit Wishbone B4 compliant slave
// Provides register access for control, status, FFT data, features, and NN weights
//
// Corrected RTL (branch fix/audit-defects).  Changes against the fabricated revision:
//   * The weight window is 0x20..0xF0, 53 word-aligned registers, one per 4 weights.
//     A word write carries up to four weight bytes selected by wb_sel_i; byte lane k of
//     the word at offset 0x20 + 4w is weight[4w+k].  The bytes are forwarded to the
//     engine's byte-wide write port over four cycles and the transfer is acknowledged
//     after the last one.  The fabricated decoder took the weight index from the byte
//     offset of the address and accepted only byte lane 0, so from the word-aligned
//     Caravel bus it reached weights 0, 4, ..., 84 only.
//   * The weight window reads back (word-wide) through the engine's read port.

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

    // NN weight loading (byte port of nn_engine)
    output reg         wt_wr_en,
    output reg  [7:0]  wt_wr_addr,
    output reg  [7:0]  wt_wr_data,

    // NN weight readback (word port of nn_engine)
    output wire [5:0]  wt_rd_waddr,
    input  wire [31:0] wt_rd_wdata,

    // Interrupt
    input  wire        classification_done,
    input  wire        alarm_irq_in,
    output reg  [2:0]  irq
);

    // --- Address map (relative to base); bits [7:0] of the address select the register ---
    localparam ADDR_CTRL         = 8'h00;
    localparam ADDR_STATUS       = 8'h04;
    localparam ADDR_CLASS_RESULT = 8'h08;
    localparam ADDR_ALARM_CFG    = 8'h0C;
    localparam ADDR_FFT_DATA     = 8'h10;
    localparam ADDR_FEATURE_DATA = 8'h14;
    localparam ADDR_IRQ_FLAGS    = 8'h18;
    localparam ADDR_CLK_DIV      = 8'h1C;
    // 0x20 .. 0xF0 : NN weights, 53 words x 4 bytes = 212 bytes
    localparam ADDR_NN_WEIGHTS_BASE = 8'h20;
    localparam ADDR_NN_WEIGHTS_END  = 8'hF0; // 0x20 + 53*4 - 4

    // --- Internal registers ---
    reg [2:0]  irq_flags;       // [0] classification done, [1] alarm, [2] reserved
    reg [2:0]  irq_enable;
    reg [4:0]  fft_auto_addr;   // Auto-incrementing FFT read address
    reg [2:0]  feat_auto_addr;  // Auto-incrementing feature read address

    // Weight write sequencer: one byte lane per cycle
    reg        wseq_active;
    reg [1:0]  wseq_lane;
    reg [7:0]  wseq_base;       // weight index of byte lane 0
    reg [31:0] wseq_data;
    reg [3:0]  wseq_sel;

    wire       wb_valid  = wb_cyc_i && wb_stb_i;
    wire [7:0] reg_addr  = wb_adr_i[7:0];
    wire [7:0] wt_off    = reg_addr - ADDR_NN_WEIGHTS_BASE;
    wire       wt_hit    = (reg_addr >= ADDR_NN_WEIGHTS_BASE) && (reg_addr <= ADDR_NN_WEIGHTS_END);

    assign wt_rd_waddr = wt_off[7:2];

    // --- IRQ flag capture ---
    always @(posedge clk) begin
        if (rst) begin
            irq_flags <= 3'd0;
        end else begin
            if (classification_done) irq_flags[0] <= 1'b1;
            if (alarm_irq_in)        irq_flags[1] <= 1'b1;
            // Clear on write to IRQ_FLAGS register (write-one-to-clear)
            if (wb_valid && !wb_ack_o && !wseq_active && wb_we_i && reg_addr == ADDR_IRQ_FLAGS)
                irq_flags <= irq_flags & ~wb_dat_i[2:0];
        end
    end

    always @(*) begin
        irq[0] = |(irq_flags & irq_enable);
        irq[1] = 1'b0;
        irq[2] = 1'b0;
    end

    // --- Wishbone transaction handling ---
    always @(posedge clk) begin
        if (rst) begin
            wb_ack_o        <= 1'b0;
            wb_dat_o        <= 32'd0;
            enable          <= 1'b0;
            clk_div         <= 16'd249;  // Default: divide by 250
            alarm_threshold <= 8'd128;
            fault_count_cfg <= 4'd3;
            irq_enable      <= 3'd0;
            fft_auto_addr   <= 5'd0;
            feat_auto_addr  <= 3'd0;
            wt_wr_en        <= 1'b0;
            wt_wr_addr      <= 8'd0;
            wt_wr_data      <= 8'd0;
            fft_rd_addr     <= 5'd0;
            feature_rd_addr <= 3'd0;
            wseq_active     <= 1'b0;
            wseq_lane       <= 2'd0;
            wseq_base       <= 8'd0;
            wseq_data       <= 32'd0;
            wseq_sel        <= 4'd0;
        end else begin
            wb_ack_o <= 1'b0;
            wt_wr_en <= 1'b0;

            if (wseq_active) begin
                // Forward byte lane wseq_lane of the latched word, then acknowledge after lane 3
                if (wseq_sel[wseq_lane]) begin
                    wt_wr_en   <= 1'b1;
                    wt_wr_addr <= wseq_base + {6'd0, wseq_lane};
                    wt_wr_data <= wseq_data[8*wseq_lane +: 8];
                end
                if (wseq_lane == 2'd3) begin
                    wseq_active <= 1'b0;
                    wb_ack_o    <= 1'b1;
                end
                wseq_lane <= wseq_lane + 2'd1;

            end else if (wb_valid && !wb_ack_o) begin
                wb_ack_o <= 1'b1;

                if (wb_we_i) begin
                    // --- Write operations ---
                    case (reg_addr)
                        ADDR_CTRL: begin
                            if (wb_sel_i[0]) enable     <= wb_dat_i[0];
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
                            fft_auto_addr <= wb_dat_i[4:0];
                            fft_rd_addr   <= wb_dat_i[4:0];
                        end
                        ADDR_FEATURE_DATA: begin
                            feat_auto_addr  <= wb_dat_i[2:0];
                            feature_rd_addr <= wb_dat_i[2:0];
                        end
                        default: begin
                            if (wt_hit) begin
                                // Weight word: latch and acknowledge after the four byte lanes
                                wb_ack_o    <= 1'b0;
                                wseq_active <= 1'b1;
                                wseq_lane   <= 2'd0;
                                wseq_base   <= {wt_off[7:2], 2'b00};
                                wseq_data   <= wb_dat_i;
                                wseq_sel    <= wb_sel_i;
                            end
                        end
                    endcase
                end else begin
                    // --- Read operations ---
                    case (reg_addr)
                        ADDR_CTRL:         wb_dat_o <= {21'd0, irq_enable, 7'd0, enable};
                        ADDR_STATUS:       wb_dat_o <= {27'd0, alarm_active, fe_busy, nn_busy, fft_busy, enable};
                        ADDR_CLASS_RESULT: wb_dat_o <= {22'd0, confidence, class_id};
                        ADDR_ALARM_CFG:    wb_dat_o <= {20'd0, fault_count_cfg, alarm_threshold};
                        ADDR_FFT_DATA: begin
                            wb_dat_o      <= {16'd0, fft_rd_data};
                            fft_auto_addr <= fft_auto_addr + 5'd1;
                            fft_rd_addr   <= fft_auto_addr + 5'd1;
                        end
                        ADDR_FEATURE_DATA: begin
                            wb_dat_o        <= {24'd0, feature_rd_data};
                            feat_auto_addr  <= feat_auto_addr + 3'd1;
                            feature_rd_addr <= feat_auto_addr + 3'd1;
                        end
                        ADDR_IRQ_FLAGS:    wb_dat_o <= {29'd0, irq_flags};
                        ADDR_CLK_DIV:      wb_dat_o <= {16'd0, clk_div};
                        default:           wb_dat_o <= wt_hit ? wt_rd_wdata : 32'd0;
                    endcase
                end
            end
        end
    end

endmodule

`default_nettype wire

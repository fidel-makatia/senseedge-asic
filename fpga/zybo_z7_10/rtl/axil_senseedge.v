// SPDX-License-Identifier: Apache-2.0
// SenseEdge on the Zybo Z7-10: AXI4-Lite slave (for the Zynq PS, which plays the part of the
// Caravel management core) bridged to the design's Wishbone slave, plus an on-chip MCP3201
// emulator so that the audit records can be replayed with no sensor attached.
//
// Address map (64 KiB window, byte offsets):
//   0x0000-0x00FF  senseedge_top Wishbone registers (the chip's own map; word aligned,
//                  byte strobes forwarded as Wishbone selects)
//   0x1000  EMU_CTRL   [0] 1 = capture reads the emulator, 0 = the Pmod ADC
//                      [1] write 1: reset the chip and the emulator (7 clocks, self-clearing;
//                          reads back 1 while active).  Replay then starts at code 0 with an
//                          empty sample buffer, so every block of windows is aligned.
//   0x1004  EMU_LEN    number of codes to play before wrapping (1..4096)
//   0x1008  ID         0x5E5E0002
//   0x100C  SW         [1:0] board switches, [15:8] next emulator code index
//   0x2000-0x5FFF  emulator code RAM, one 12-bit code per word (4096 words)
`default_nettype none
module axil_senseedge (
    (* X_INTERFACE_INFO = "xilinx.com:signal:clock:1.0 s_axi_aclk CLK" *)
    (* X_INTERFACE_PARAMETER = "ASSOCIATED_BUSIF s_axi, ASSOCIATED_RESET s_axi_aresetn, FREQ_HZ 20000000" *)
    input  wire        s_axi_aclk,
    (* X_INTERFACE_INFO = "xilinx.com:signal:reset:1.0 s_axi_aresetn RST" *)
    (* X_INTERFACE_PARAMETER = "POLARITY ACTIVE_LOW" *)
    input  wire        s_axi_aresetn,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 s_axi AWADDR" *)
    input  wire [15:0] s_axi_awaddr,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 s_axi AWVALID" *)
    input  wire        s_axi_awvalid,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 s_axi AWREADY" *)
    output reg         s_axi_awready,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 s_axi WDATA" *)
    input  wire [31:0] s_axi_wdata,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 s_axi WSTRB" *)
    input  wire [3:0]  s_axi_wstrb,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 s_axi WVALID" *)
    input  wire        s_axi_wvalid,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 s_axi WREADY" *)
    output reg         s_axi_wready,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 s_axi BRESP" *)
    output wire [1:0]  s_axi_bresp,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 s_axi BVALID" *)
    output reg         s_axi_bvalid,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 s_axi BREADY" *)
    input  wire        s_axi_bready,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 s_axi ARADDR" *)
    input  wire [15:0] s_axi_araddr,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 s_axi ARVALID" *)
    input  wire        s_axi_arvalid,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 s_axi ARREADY" *)
    output reg         s_axi_arready,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 s_axi RDATA" *)
    output reg  [31:0] s_axi_rdata,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 s_axi RRESP" *)
    output wire [1:0]  s_axi_rresp,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 s_axi RVALID" *)
    output reg         s_axi_rvalid,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 s_axi RREADY" *)
    input  wire        s_axi_rready,

    // board
    input  wire        spi_miso,      // Pmod JE1: ADC DOUT
    output wire        spi_sclk,      // Pmod JE2: ADC CLK
    output wire        spi_cs_n,      // Pmod JE3: ADC CS
    output wire [3:0]  led,           // LD0 alarm, LD1 status, LD2 emulator selected, LD3 toggles per decision
    input  wire [1:0]  sw,
    output wire        irq            // decision / alarm interrupt (senseedge irq[0])
);
    assign s_axi_bresp = 2'b00;
    assign s_axi_rresp = 2'b00;
    wire clk = s_axi_aclk;
    wire rst = ~s_axi_aresetn;

    // ---------------- local registers and emulator ----------------
    reg         emu_sel;
    reg  [2:0]  soft_rst;                              // chip + emulator reset pulse
    wire        chip_rst = rst | (soft_rst != 3'd0);
    reg  [12:0] emu_len;
    wire        emu_miso;
    wire [12:0] emu_idx;
    reg         emu_wr_en;
    reg  [11:0] emu_wr_addr;
    reg  [11:0] emu_wr_data;

    // ---------------- Wishbone side ----------------
    reg         wb_cyc, wb_stb, wb_we;
    reg  [3:0]  wb_sel;
    reg  [31:0] wb_adr, wb_dat_w;
    wire        wb_ack;
    wire [31:0] wb_dat_r;

    // ---------------- AXI-Lite slave FSM ----------------
    localparam S_IDLE = 3'd0, S_WB_W = 3'd1, S_B = 3'd2, S_WB_R = 3'd3, S_R = 3'd4;
    reg [2:0]  state;
    reg [15:0] addr;
    reg [31:0] wdata;
    reg [3:0]  wstrb;
    wire [15:0] ram_off = s_axi_awaddr - 16'h2000;     // code RAM index = (offset - 0x2000) / 4

    always @(posedge clk) begin
        if (rst) begin
            state <= S_IDLE; s_axi_awready <= 1'b0; s_axi_wready <= 1'b0; s_axi_bvalid <= 1'b0;
            s_axi_arready <= 1'b0; s_axi_rvalid <= 1'b0; s_axi_rdata <= 32'd0;
            wb_cyc <= 1'b0; wb_stb <= 1'b0; wb_we <= 1'b0; wb_sel <= 4'd0; wb_adr <= 32'd0; wb_dat_w <= 32'd0;
            emu_sel <= 1'b0; soft_rst <= 3'd0; emu_len <= 13'd64; emu_wr_en <= 1'b0; emu_wr_addr <= 12'd0; emu_wr_data <= 12'd0;
            addr <= 16'd0; wdata <= 32'd0; wstrb <= 4'd0;
        end else begin
            s_axi_awready <= 1'b0; s_axi_wready <= 1'b0; s_axi_arready <= 1'b0;
            emu_wr_en <= 1'b0;
            if (soft_rst != 3'd0) soft_rst <= soft_rst - 3'd1;
            case (state)
                S_IDLE: begin
                    if (s_axi_awvalid && s_axi_wvalid) begin
                        s_axi_awready <= 1'b1; s_axi_wready <= 1'b1;
                        addr <= s_axi_awaddr; wdata <= s_axi_wdata; wstrb <= s_axi_wstrb;
                        if (s_axi_awaddr[15:12] == 4'h0) begin
                            wb_cyc <= 1'b1; wb_stb <= 1'b1; wb_we <= 1'b1; wb_sel <= s_axi_wstrb;
                            wb_adr <= {16'h3000, s_axi_awaddr}; wb_dat_w <= s_axi_wdata;
                            state <= S_WB_W;
                        end else begin
                            // local write, completes next cycle
                            if (s_axi_awaddr == 16'h1000) begin
                                emu_sel <= s_axi_wdata[0];
                                if (s_axi_wdata[1]) soft_rst <= 3'd7;
                            end
                            if (s_axi_awaddr == 16'h1004) emu_len <= s_axi_wdata[12:0];
                            if (s_axi_awaddr >= 16'h2000 && s_axi_awaddr < 16'h6000) begin
                                emu_wr_en <= 1'b1; emu_wr_addr <= ram_off[13:2]; emu_wr_data <= s_axi_wdata[11:0];
                            end
                            s_axi_bvalid <= 1'b1; state <= S_B;
                        end
                    end else if (s_axi_arvalid) begin
                        s_axi_arready <= 1'b1; addr <= s_axi_araddr;
                        if (s_axi_araddr[15:12] == 4'h0) begin
                            wb_cyc <= 1'b1; wb_stb <= 1'b1; wb_we <= 1'b0; wb_sel <= 4'hF;
                            wb_adr <= {16'h3000, s_axi_araddr};
                            state <= S_WB_R;
                        end else begin
                            case (s_axi_araddr)
                                16'h1000: s_axi_rdata <= {30'd0, soft_rst != 3'd0, emu_sel};
                                16'h1004: s_axi_rdata <= {19'd0, emu_len};
                                16'h1008: s_axi_rdata <= 32'h5E5E0002;
                                16'h100C: s_axi_rdata <= {16'd0, emu_idx[7:0], 6'd0, sw};
                                default:  s_axi_rdata <= 32'd0;
                            endcase
                            s_axi_rvalid <= 1'b1; state <= S_R;
                        end
                    end
                end
                S_WB_W: begin
                    if (wb_ack) begin
                        wb_cyc <= 1'b0; wb_stb <= 1'b0; wb_we <= 1'b0;
                        s_axi_bvalid <= 1'b1; state <= S_B;
                    end
                end
                S_B: begin
                    if (s_axi_bready) begin s_axi_bvalid <= 1'b0; state <= S_IDLE; end
                end
                S_WB_R: begin
                    if (wb_ack) begin
                        wb_cyc <= 1'b0; wb_stb <= 1'b0;
                        s_axi_rdata <= wb_dat_r; s_axi_rvalid <= 1'b1; state <= S_R;
                    end
                end
                S_R: begin
                    if (s_axi_rready) begin s_axi_rvalid <= 1'b0; state <= S_IDLE; end
                end
                default: state <= S_IDLE;
            endcase
        end
    end

    // ---------------- the chip ----------------
    wire [127:0] la_out;
    wire [15:0]  io_out, io_oeb;
    wire [2:0]   se_irq;
    wire         miso_mux = emu_sel ? emu_miso : spi_miso;
    wire [15:0]  io_in = {15'd0, miso_mux};

    senseedge_top u_se (
        .wb_clk_i(clk), .wb_rst_i(chip_rst),
        .wbs_stb_i(wb_stb), .wbs_cyc_i(wb_cyc), .wbs_we_i(wb_we), .wbs_sel_i(wb_sel),
        .wbs_dat_i(wb_dat_w), .wbs_adr_i(wb_adr), .wbs_ack_o(wb_ack), .wbs_dat_o(wb_dat_r),
        .la_data_in(128'd0), .la_data_out(la_out), .la_oenb({128{1'b1}}),
        .io_in(io_in), .io_out(io_out), .io_oeb(io_oeb), .irq(se_irq));

    assign spi_sclk = io_out[1];
    assign spi_cs_n = io_out[2];
    assign irq      = se_irq[0];

    mcp3201_emu #(.AW(12)) u_emu (
        .clk(clk), .rst(chip_rst), .sclk(io_out[1]), .cs_n(io_out[2]), .miso(emu_miso),
        .wr_en(emu_wr_en), .wr_addr(emu_wr_addr), .wr_data(emu_wr_data), .len(emu_len), .play_idx(emu_idx));

    // LEDs: alarm, status, emulator selected, heartbeat on every decision
    reg hb;
    always @(posedge clk) if (chip_rst) hb <= 1'b0; else if (la_out[15]) hb <= ~hb;
    assign led = {hb, emu_sel, io_out[4], io_out[3]};
endmodule
`default_nettype wire

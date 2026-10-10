// Audit harness: the corrected RTL observed only through the Wishbone slave, as the
// Caravel management core sees it.  Word-aligned addresses, byte selects.
//   1. load 53 weight words from weights.hex, read every word back          -> "W w value"
//   2. partial-select write to word 0 (lane 1 only), read back              -> "P value"
//   3. write 0xFFFFFFFF above the window (0xF4), read 0xF4 and word 52 back -> "X v0 v52"
//   4. for each window: bins and features over FFT_DATA / FEATURE_DATA, then CLASS
//                                                                           -> "M w ...", "F w ...", "C w class conf"
`timescale 1ns/1ps
module tb_bus;
  reg clk = 0, rst = 1; always #25 clk = ~clk;
  reg cyc = 0, stb = 0, we = 0; reg [3:0] sel = 0; reg [31:0] dat = 0, adr = 0;
  wire ack; wire [31:0] dato; wire [127:0] la_out; wire [15:0] io_out, io_oeb; wire [2:0] irq;
  reg miso = 1'b0; wire [15:0] io_in = {15'd0, miso};
  senseedge_top dut (.wb_clk_i(clk), .wb_rst_i(rst), .wbs_stb_i(stb), .wbs_cyc_i(cyc), .wbs_we_i(we),
    .wbs_sel_i(sel), .wbs_dat_i(dat), .wbs_adr_i(adr), .wbs_ack_o(ack), .wbs_dat_o(dato),
    .la_data_in(128'd0), .la_data_out(la_out), .la_oenb({128{1'b1}}), .io_in(io_in), .io_out(io_out),
    .io_oeb(io_oeb), .irq(irq));
  wire sclk = io_out[1], csn = io_out[2];
  wire nn_done = la_out[15];

  // MCP3201 model from the data sheet timing (same as the pre-silicon audit)
  reg [11:0] code [0:64*256-1];
  integer sidx = 0, fall = 0; reg [11:0] cur;
  always @(negedge csn) if (!rst) begin cur = code[sidx]; sidx = sidx + 1; fall = 0; miso <= 1'b1; end
  always @(negedge sclk) if (!csn) begin
    fall = fall + 1;
    if (fall == 2)                    miso <= #5 1'b0;
    else if (fall >= 3 && fall <= 14) miso <= #5 cur[14 - fall];
    else if (fall >= 15)              miso <= #5 cur[fall - 14];
  end

  task wb_write_sel(input [31:0] a, input [31:0] d, input [3:0] s);
    begin @(posedge clk); cyc <= 1; stb <= 1; we <= 1; sel <= s; adr <= a; dat <= d;
      @(posedge clk); wait (ack === 1'b1); @(posedge clk); cyc <= 0; stb <= 0; we <= 0; @(posedge clk); end
  endtask
  task wb_write(input [31:0] a, input [31:0] d); begin wb_write_sel(a, d, 4'hF); end endtask
  reg [31:0] rd;
  task wb_read(input [31:0] a);
    begin @(posedge clk); cyc <= 1; stb <= 1; we <= 0; sel <= 4'hF; adr <= a;
      @(posedge clk); wait (ack === 1'b1); rd = dato; @(posedge clk); cyc <= 0; stb <= 0; @(posedge clk); end
  endtask

  reg [7:0] wts [0:211];
  integer nwin, w, i, fd, div;
`ifdef DUMP
  initial begin
    $dumpfile("dump.vcd");
    $dumpvars(0, dut.wb_rst_i, dut.la_data_out, dut.fft_start_reg, dut.fe_start_reg, dut.nn_start_reg,
                 dut.fft_mag_addr_mux, dut.feature_addr_mux, dut.fft_busy, dut.fe_busy, dut.nn_busy,
                 dut.fft_done, dut.fe_done, dut.nn_done, dut.samples_valid, dut.enable,
                 dut.u_fft.state, dut.u_fft.stage, dut.u_fft.bf_idx, dut.u_fft.grp_idx, dut.u_fft.idx_p,
                 dut.u_fft.idx_q, dut.u_fft.tw_idx, dut.u_fft.mag_cnt, dut.u_fft.num_groups,
                 dut.u_feature.state, dut.u_feature.scan_idx, dut.u_feature.mag_addr,
                 dut.u_nn.state, dut.u_spi_adc.state, dut.u_spi_adc.bit_cnt, dut.u_spi_adc.spi_cs_n);
  end
`endif
  initial begin
    $readmemh("codes.hex", code);
    $readmemh("weights.hex", wts);
    if (!$value$plusargs("nwin=%d", nwin)) nwin = 1;
    if (code[nwin*64-1] === 12'bxxxxxxxxxxxx) begin
      $display("tb_bus: codes.hex holds fewer than %0d windows", nwin); $finish;
    end
    if (!$value$plusargs("div=%d", div)) div = 4;
    fd = $fopen("bus_out.txt", "w");
    repeat (10) @(posedge clk); rst = 0; repeat (5) @(posedge clk);

    // 1. weights, word by word, through the word-aligned bus
    for (w = 0; w < 53; w = w + 1)
      wb_write(32'h3000_0020 + 4*w, {wts[4*w+3], wts[4*w+2], wts[4*w+1], wts[4*w]});
    for (w = 0; w < 53; w = w + 1) begin wb_read(32'h3000_0020 + 4*w); $fwrite(fd, "W %0d %0d\n", w, rd); end

    // 2. partial select: only byte lane 1 of word 0 changes
    wb_write_sel(32'h3000_0020, 32'hA5A5A5A5, 4'b0010);
    wb_read(32'h3000_0020); $fwrite(fd, "P %0d\n", rd);
    wb_write(32'h3000_0020, {wts[3], wts[2], wts[1], wts[0]});   // restore

    // 3. above the window: no effect, reads zero
    wb_write(32'h3000_00F4, 32'hFFFFFFFF);
    wb_read(32'h3000_00F4); i = rd; wb_read(32'h3000_00F0); $fwrite(fd, "X %0d %0d\n", i, rd);

    // 4. run windows
    wb_write(32'h3000_001C, div);
    wb_write(32'h3000_0000, 32'd1);
    for (w = 0; w < nwin; w = w + 1) begin
      @(posedge nn_done); #1;
      wb_write(32'h3000_0010, 0);
      $fwrite(fd, "M %0d", w);
      for (i = 0; i < 32; i = i + 1) begin wb_read(32'h3000_0010); $fwrite(fd, " %0d", rd[15:0]); end
      wb_write(32'h3000_0014, 0);
      $fwrite(fd, "\nF %0d", w);
      for (i = 0; i < 8; i = i + 1) begin wb_read(32'h3000_0014); $fwrite(fd, " %0d", rd[7:0]); end
      wb_read(32'h3000_0008);
      $fwrite(fd, "\nC %0d %0d %0d\n", w, rd[1:0], rd[9:2]);
    end
    $fclose(fd); $finish;
  end
endmodule

// Audit harness: bus-protocol and operational stress of senseedge_top through the Wishbone
// slave only.  Scenarios (plusarg +scn=N), all with weights from weights.hex and the first
// window record from codes.hex; every scenario ends by printing the window's bins, features
// and class exactly as tb_bus does, so the Python side compares them with the reference.
//   0  baseline: one idle cycle between transactions (as tb_bus)
//   1  back-to-back master: cyc/stb stay high, the next transaction follows the ack directly
//   2  weight words rewritten (same values) while the chain is capturing the window
//   3  reset asserted in the middle of a window, then the window is re-run
//   4  FFT_DATA / FEATURE_DATA / CLASS read repeatedly while the chain is busy
//   5  weight write with sel = 0000 and a write to an unmapped offset (0x1E8): no stall
//   6  divider 0 (clock at 36 x f_s) instead of 4
// Output lines: "M 0 ...", "F 0 ...", "C 0 class conf", "I flags_before flags_after", "T clocks".
`timescale 1ns/1ps
module tb_stress;
  reg clk = 0, rst = 1; always #25 clk = ~clk;
  reg cyc = 0, stb = 0, we = 0; reg [3:0] sel = 0; reg [31:0] dat = 0, adr = 0;
  wire ack; wire [31:0] dato; wire [127:0] la_out; wire [15:0] io_out, io_oeb; wire [2:0] irq;
  reg miso = 1'b0; wire [15:0] io_in = {15'd0, miso};
  senseedge_top dut (.wb_clk_i(clk), .wb_rst_i(rst), .wbs_stb_i(stb), .wbs_cyc_i(cyc), .wbs_we_i(we),
    .wbs_sel_i(sel), .wbs_dat_i(dat), .wbs_adr_i(adr), .wbs_ack_o(ack), .wbs_dat_o(dato),
    .la_data_in(128'd0), .la_data_out(la_out), .la_oenb({128{1'b1}}), .io_in(io_in), .io_out(io_out),
    .io_oeb(io_oeb), .irq(irq));
  wire sclk = io_out[1], csn = io_out[2];
  wire nn_done = la_out[15], samples_valid = la_out[22];

  // MCP3201 model (data-sheet timing); the same record is replayed for every window
  reg [11:0] code [0:63];
  integer sidx = 0, fall = 0; reg [11:0] cur;
  always @(negedge csn) if (!rst) begin cur = code[sidx % 64]; sidx = sidx + 1; fall = 0; miso <= 1'b1; end
  always @(negedge sclk) if (!csn) begin
    fall = fall + 1;
    if (fall == 2)                    miso <= #5 1'b0;
    else if (fall >= 3 && fall <= 14) miso <= #5 cur[14 - fall];
    else if (fall >= 15)              miso <= #5 cur[fall - 14];
  end

  integer scn, i, w, fd, t0;
  reg [31:0] rd, flags0;
  reg b2b = 0;                            // back-to-back master
  // one transaction; with b2b the bus is not released between transactions
  // A classic Wishbone master samples ack on a clock edge and, in that same cycle, either
  // negates stb or presents the next transaction.  With b2b the next xfer drives its signals
  // at the edge after ack and never releases the bus; without it the bus idles one cycle.
  task xfer(input wr, input [31:0] a, input [31:0] d, input [3:0] s);
    begin
      @(posedge clk); cyc <= 1; stb <= 1; we <= wr; sel <= s; adr <= a; dat <= d;
      @(posedge clk); wait (ack === 1'b1); rd = dato;
      if (!b2b) begin @(posedge clk); cyc <= 0; stb <= 0; we <= 0; @(posedge clk); end
    end
  endtask
  task release_bus; begin @(posedge clk); cyc <= 0; stb <= 0; we <= 0; @(posedge clk); end endtask
  task wb_write(input [31:0] a, input [31:0] d); begin xfer(1, a, d, 4'hF); end endtask
  task wb_read(input [31:0] a); begin xfer(0, a, 0, 4'hF); end endtask

  reg [7:0] wts [0:211];
  task load_weights; begin
    for (w = 0; w < 53; w = w + 1) wb_write(32'h3000_0020 + 4*w, {wts[4*w+3], wts[4*w+2], wts[4*w+1], wts[4*w]});
  end endtask
  task dump_window; begin
    wb_write(32'h3000_0010, 0); $fwrite(fd, "M 0");
    for (i = 0; i < 32; i = i + 1) begin wb_read(32'h3000_0010); $fwrite(fd, " %0d", rd[15:0]); end
    wb_write(32'h3000_0014, 0); $fwrite(fd, "\nF 0");
    for (i = 0; i < 8; i = i + 1) begin wb_read(32'h3000_0014); $fwrite(fd, " %0d", rd[7:0]); end
    wb_read(32'h3000_0008); $fwrite(fd, "\nC 0 %0d %0d\n", rd[1:0], rd[9:2]);
  end endtask

  initial begin
    $readmemh("codes.hex", code);
    $readmemh("weights.hex", wts);
    if (!$value$plusargs("scn=%d", scn)) scn = 0;
    if (code[63] === 12'bxxxxxxxxxxxx) begin $display("tb_stress: codes.hex is short"); $finish; end
    fd = $fopen("stress_out.txt", "w");
    repeat (10) @(posedge clk); rst = 0; repeat (5) @(posedge clk);
    if (scn == 1) b2b = 1;

    load_weights();
    wb_write(32'h3000_001C, (scn == 6) ? 0 : 4);
    if (scn == 5) begin
      xfer(1, 32'h3000_0020, 32'hDEADBEEF, 4'b0000);      // weight word with no byte lane: acknowledged, no change
      xfer(1, 32'h3000_00F8, 32'hFFFFFFFF, 4'hF);         // above the window: acknowledged, ignored
      xfer(1, 32'h3000_0000, 32'h0000_0700, 4'b0010);     // CTRL lane 1 only: irq_enable set, enable untouched
      xfer(0, 32'h3000_0020, 0, 4'hF);                    // word 0 must still hold the loaded weights
      if (rd !== {wts[3], wts[2], wts[1], wts[0]}) $fwrite(fd, "E word0 changed\n");
      xfer(0, 32'h3000_0000, 0, 4'hF);
      if (rd !== 32'h0000_0700) $fwrite(fd, "E ctrl %0h\n", rd);
      wb_write(32'h3000_0000, 32'h0000_0000);             // irq_enable back to 0, enable still 0
    end
    wb_write(32'h3000_0000, 32'd1); t0 = $time;
    if (b2b) release_bus();                               // a master releases the bus when it has nothing to send

    if (scn == 2) begin
      // rewrite the weights while the window is being captured (well before the network runs)
      repeat (200) @(posedge clk);
      load_weights();
      if (b2b) release_bus();
    end
    if (scn == 3) begin
      repeat (1500) @(posedge clk);                       // inside the window
      if (b2b) release_bus();
      rst = 1; repeat (3) @(posedge clk); rst = 0; repeat (3) @(posedge clk);
      sidx = 0;                                           // the ADC model restarts with the design
      load_weights();
      wb_write(32'h3000_001C, 4);
      wb_write(32'h3000_0000, 32'd1); t0 = $time;
      if (b2b) release_bus();
    end
    if (scn == 4) begin
      // hammer the read ports while the chain is busy; the window result must be unaffected
      @(posedge samples_valid);
      for (i = 0; i < 40; i = i + 1) begin wb_read(32'h3000_0010); wb_read(32'h3000_0014); wb_read(32'h3000_0008); end
      if (b2b) release_bus();
    end
    @(posedge nn_done); #1;
    $fwrite(fd, "T %0d\n", ($time - t0) / 50);
    wb_read(32'h3000_0018); flags0 = rd;                  // decision flag set
    wb_write(32'h3000_0018, 32'h1);                       // write-one-to-clear
    wb_read(32'h3000_0018); $fwrite(fd, "I %0d %0d\n", flags0[0], rd[0]);
    dump_window();
    if (b2b) release_bus();
    $fclose(fd); $finish;
  end
  // watchdog
  initial begin #200_000_000; $display("STRESS TIMEOUT scn=%0d", scn); $finish; end
endmodule

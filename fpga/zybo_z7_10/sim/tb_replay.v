// Replay simulation of the Zybo deployment: an AXI-Lite master drives axil_senseedge exactly
// as the host-mode protocol of sw/senseedge_zybo.c does for a dataset replay.  For every block
// of up to `blk` windows: write the codes into the emulator RAM, set EMU_LEN, soft-reset the
// chip and the emulator (EMU_CTRL bit 1), reload the 53 weight words, program the divider and
// the alarm, enable, and for every window wait for the decision flag, read CLASS, STATUS, the
// 8 features and the 32 bins, and clear the flag.  Output: replay_out.txt, checked by
// run_replay_check.py against the Python reference chain.
//   plusargs: +nwin=N (windows in codes.hex, <= 1024) +div=D +blk=B +thr=T +ncons=K
//   files:    codes.hex (3 hex digits per code), weights.hex (2 hex digits per byte, 212)
`timescale 1ns/1ps
module tb_replay;
  reg clk = 0; always #25 clk = ~clk;
  reg aresetn = 0;
  reg [15:0] awaddr = 0, araddr = 0; reg awvalid = 0, wvalid = 0, bready = 0, arvalid = 0, rready = 0;
  reg [31:0] wdata = 0; reg [3:0] wstrb = 0;
  wire awready, wready, bvalid, arready, rvalid; wire [1:0] bresp, rresp; wire [31:0] rdata;
  wire spi_sclk, spi_cs_n; wire [3:0] led; wire irq;
  axil_senseedge dut (.s_axi_aclk(clk), .s_axi_aresetn(aresetn),
    .s_axi_awaddr(awaddr), .s_axi_awvalid(awvalid), .s_axi_awready(awready),
    .s_axi_wdata(wdata), .s_axi_wstrb(wstrb), .s_axi_wvalid(wvalid), .s_axi_wready(wready),
    .s_axi_bresp(bresp), .s_axi_bvalid(bvalid), .s_axi_bready(bready),
    .s_axi_araddr(araddr), .s_axi_arvalid(arvalid), .s_axi_arready(arready),
    .s_axi_rdata(rdata), .s_axi_rresp(rresp), .s_axi_rvalid(rvalid), .s_axi_rready(rready),
    .spi_miso(1'b0), .spi_sclk(spi_sclk), .spi_cs_n(spi_cs_n), .led(led), .sw(2'b00), .irq(irq));

  task axi_write(input [15:0] a, input [31:0] d);
    begin
      @(posedge clk); awaddr <= a; wdata <= d; wstrb <= 4'hF; awvalid <= 1; wvalid <= 1; bready <= 1;
      wait (awready && wready); @(posedge clk); awvalid <= 0; wvalid <= 0;
      wait (bvalid); @(posedge clk); bready <= 0; @(posedge clk);
    end
  endtask
  reg [31:0] rd;
  task axi_read(input [15:0] a);
    begin
      @(posedge clk); araddr <= a; arvalid <= 1; rready <= 1;
      wait (arready); @(posedge clk); arvalid <= 0;
      wait (rvalid); rd = rdata; @(posedge clk); rready <= 0; @(posedge clk);
    end
  endtask

  reg [11:0] code [0:64*1024-1];
  reg [7:0]  wimg [0:211];
  integer nwin, div, blk, thr, ncons, g, nb, w, i, fd, t, werr, cls, conf, alarm;
  initial begin
    $readmemh("codes.hex", code);
    $readmemh("weights.hex", wimg);
    if (!$value$plusargs("nwin=%d", nwin)) nwin = 1;
    if (!$value$plusargs("div=%d", div)) div = 9;
    if (!$value$plusargs("blk=%d", blk)) blk = 64;
    if (!$value$plusargs("thr=%d", thr)) thr = 100;
    if (!$value$plusargs("ncons=%d", ncons)) ncons = 3;
    fd = $fopen("replay_out.txt", "w");
    repeat (5) @(posedge clk); aresetn = 1; repeat (5) @(posedge clk);
    axi_read(16'h1008); $fwrite(fd, "ID %h\n", rd);
    for (g = 0; g < nwin; g = g + blk) begin
      nb = (nwin - g < blk) ? nwin - g : blk;
      for (i = 0; i < nb * 64; i = i + 1) axi_write(16'h2000 + 4 * i, code[g * 64 + i]);
      axi_write(16'h1004, nb * 64);
      axi_write(16'h1000, 32'h3);                          // emulator selected + soft reset
      repeat (12) @(posedge clk);
      axi_read(16'h1000); if (rd[1]) begin $display("FAIL: soft reset still active"); end
      werr = 0;
      for (w = 0; w < 53; w = w + 1) axi_write(16'h0020 + 4 * w, {wimg[4*w+3], wimg[4*w+2], wimg[4*w+1], wimg[4*w]});
      for (w = 0; w < 53; w = w + 1) begin
        axi_read(16'h0020 + 4 * w);
        if (rd !== {wimg[4*w+3], wimg[4*w+2], wimg[4*w+1], wimg[4*w]}) werr = werr + 1;
      end
      $fwrite(fd, "B %0d %0d %0d\n", g, nb, werr);          // block start, windows, weight read-back errors
      axi_write(16'h001C, div);
      axi_write(16'h000C, (ncons << 8) | thr);
      axi_write(16'h0018, 3);
      axi_write(16'h0000, 1);
      for (w = 0; w < nb; w = w + 1) begin
        for (t = 0; t < 400000; t = t + 1) begin axi_read(16'h0018); if (rd[0]) t = 400000; end
        if (!rd[0]) begin $display("FAIL: no decision for window %0d", g + w); $fwrite(fd, "TIMEOUT %0d\n", g + w); end
        axi_read(16'h0008); cls = rd[1:0]; conf = rd[9:2];
        axi_read(16'h0004); alarm = rd[4];
        $fwrite(fd, "W %0d %0d %0d %0d\n", g + w, cls, conf, alarm);
        axi_write(16'h0014, 0); $fwrite(fd, "F %0d", g + w);
        for (i = 0; i < 8; i = i + 1) begin axi_read(16'h0014); $fwrite(fd, " %0d", rd[7:0]); end
        axi_write(16'h0010, 0); $fwrite(fd, "\nM %0d", g + w);
        for (i = 0; i < 32; i = i + 1) begin axi_read(16'h0010); $fwrite(fd, " %0d", rd[15:0]); end
        $fwrite(fd, "\n");
        axi_write(16'h0018, 1);                              // clear the decision flag
      end
      axi_write(16'h0000, 0);
    end
    $fclose(fd);
    $display("tb_replay: %0d windows done", nwin);
    $finish;
  end
  initial begin #4_000_000_000; $display("TIMEOUT"); $finish; end
endmodule

// Simulation of the Zybo deployment: an AXI4-Lite master drives axil_senseedge exactly as the
// bare-metal program on the Zynq PS does (sw/senseedge_zybo.c): write the bin-8 record into the
// emulator, select it, load the diagonal weight image as 53 words, read them back, configure,
// enable, wait for the decision flag, read bins, features and class, and compare with the
// values the RTL audit established (verilog/dv/audit): bins[8]=16010, bins[24]=10, features
// {0,62,0,0,64,62,1,62}, class 1, confidence 255.  Then a byte-strobe write and a second window.
`timescale 1ns/1ps
module tb_axil_senseedge;
  reg clk = 0; always #25 clk = ~clk;          // 20 MHz, as FCLK_CLK0
  reg aresetn = 0;
  reg [15:0] awaddr = 0, araddr = 0; reg awvalid = 0, wvalid = 0, bready = 0, arvalid = 0, rready = 0;
  reg [31:0] wdata = 0; reg [3:0] wstrb = 0;
  wire awready, wready, bvalid, arready, rvalid; wire [1:0] bresp, rresp; wire [31:0] rdata;
  reg spi_miso = 0; wire spi_sclk, spi_cs_n; wire [3:0] led; wire irq;
  axil_senseedge dut (.s_axi_aclk(clk), .s_axi_aresetn(aresetn),
    .s_axi_awaddr(awaddr), .s_axi_awvalid(awvalid), .s_axi_awready(awready),
    .s_axi_wdata(wdata), .s_axi_wstrb(wstrb), .s_axi_wvalid(wvalid), .s_axi_wready(wready),
    .s_axi_bresp(bresp), .s_axi_bvalid(bvalid), .s_axi_bready(bready),
    .s_axi_araddr(araddr), .s_axi_arvalid(arvalid), .s_axi_arready(arready),
    .s_axi_rdata(rdata), .s_axi_rresp(rresp), .s_axi_rvalid(rvalid), .s_axi_rready(rready),
    .spi_miso(spi_miso), .spi_sclk(spi_sclk), .spi_cs_n(spi_cs_n), .led(led), .sw(2'b00), .irq(irq));

  task axi_write(input [15:0] a, input [31:0] d, input [3:0] s);
    begin
      @(posedge clk); awaddr <= a; wdata <= d; wstrb <= s; awvalid <= 1; wvalid <= 1; bready <= 1;
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

  integer i, w, errors = 0, t;
  reg [11:0] rec [0:63];
  reg [7:0]  wimg [0:211];
  reg [15:0] exp_bins [0:31];
  reg [7:0]  exp_feat [0:7];
  initial begin
    for (i = 0; i < 64; i = i + 1) case (i % 8)
      0: rec[i] = 2048; 1: rec[i] = 2402; 2: rec[i] = 2548; 3: rec[i] = 2402;
      4: rec[i] = 2048; 5: rec[i] = 1694; 6: rec[i] = 1548; 7: rec[i] = 1694; endcase
    for (i = 0; i < 212; i = i + 1) wimg[i] = 0;
    wimg[0] = 127; wimg[9] = 127; wimg[18] = 127; wimg[27] = 127;
    wimg[144] = 127; wimg[161] = 127; wimg[178] = 127; wimg[195] = 127;
    for (i = 0; i < 32; i = i + 1) exp_bins[i] = 0; exp_bins[8] = 16010; exp_bins[24] = 10;
    exp_feat[0] = 0; exp_feat[1] = 62; exp_feat[2] = 0; exp_feat[3] = 0;
    exp_feat[4] = 64; exp_feat[5] = 62; exp_feat[6] = 1; exp_feat[7] = 62;

    repeat (5) @(posedge clk); aresetn = 1; repeat (5) @(posedge clk);
    axi_read(16'h1008); if (rd !== 32'h5E5E0002) begin $display("FAIL id %h", rd); errors = errors + 1; end
    // emulator record
    for (i = 0; i < 64; i = i + 1) axi_write(16'h2000 + 4*i, rec[i], 4'hF);
    axi_write(16'h1004, 64, 4'hF);
    axi_write(16'h1000, 1, 4'hF);
    // weights, word by word, and read-back
    for (w = 0; w < 53; w = w + 1) axi_write(16'h0020 + 4*w, {wimg[4*w+3], wimg[4*w+2], wimg[4*w+1], wimg[4*w]}, 4'hF);
    for (w = 0; w < 53; w = w + 1) begin
      axi_read(16'h0020 + 4*w);
      if (rd !== {wimg[4*w+3], wimg[4*w+2], wimg[4*w+1], wimg[4*w]}) begin $display("FAIL weight word %0d", w); errors = errors + 1; end
    end
    // byte strobe: only lane 1 of word 0 changes, then restore
    axi_write(16'h0020, 32'hA5A5A5A5, 4'b0010); axi_read(16'h0020);
    if (rd !== 32'h0000A57F) begin $display("FAIL strobe write %h", rd); errors = errors + 1; end
    axi_write(16'h0020, {wimg[3], wimg[2], wimg[1], wimg[0]}, 4'hF);
    // configure and run
    axi_write(16'h001C, 4, 4'hF);                 // divider 4
    axi_write(16'h000C, (3 << 8) | 100, 4'hF);    // threshold 100, N = 3
    axi_write(16'h0018, 3, 4'hF);                 // clear flags
    axi_write(16'h0000, 1, 4'hF);                 // enable
    for (t = 0; t < 200000; t = t + 1) begin axi_read(16'h0018); if (rd[0]) t = 200000; end
    if (!rd[0]) begin $display("FAIL no decision"); errors = errors + 1; end
    axi_write(16'h0000, 0, 4'hF);                 // stop so that the read-back is of this window
    axi_write(16'h0010, 0, 4'hF);
    for (i = 0; i < 32; i = i + 1) begin axi_read(16'h0010); if (rd[15:0] !== exp_bins[i]) begin $display("FAIL bin %0d = %0d", i, rd[15:0]); errors = errors + 1; end end
    axi_write(16'h0014, 0, 4'hF);
    for (i = 0; i < 8; i = i + 1) begin axi_read(16'h0014); if (rd[7:0] !== exp_feat[i]) begin $display("FAIL feature %0d = %0d", i, rd[7:0]); errors = errors + 1; end end
    axi_read(16'h0008);
    if (rd[1:0] !== 2'd1 || rd[9:2] !== 8'd255) begin $display("FAIL class %0d conf %0d", rd[1:0], rd[9:2]); errors = errors + 1; end
    $display("window 1: class %0d conf %0d, led = %b", rd[1:0], rd[9:2], led);
    // a second window on the same record, from the external-ADC path? no: keep the emulator, re-enable
    axi_write(16'h0018, 3, 4'hF); axi_write(16'h0000, 1, 4'hF);
    for (t = 0; t < 200000; t = t + 1) begin axi_read(16'h0018); if (rd[0]) t = 200000; end
    axi_read(16'h0008);
    if (rd[1:0] !== 2'd1 || rd[9:2] !== 8'd255) begin $display("FAIL second window class %0d", rd[1:0]); errors = errors + 1; end
    axi_read(16'h100C); $display("SW/idx register = %h", rd);
    if (errors == 0) $display("ZYBO WRAPPER: PASS (id, record, 53 weight words, byte strobe, bins, features, class on two windows)");
    else $display("ZYBO WRAPPER: FAIL with %0d errors", errors);
    $finish;
  end
  initial begin #60_000_000; $display("TIMEOUT"); $finish; end
endmodule

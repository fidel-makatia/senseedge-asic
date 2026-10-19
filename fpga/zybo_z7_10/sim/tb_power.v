// Switching-activity capture for the power estimate (scripts/power_saif.tcl): the wrapper runs
// recorded vibration windows from the emulator exactly as on the board, and xsim writes a SAIF
// that report_power annotates onto the implemented design.  The instance path mirrors the
// implemented hierarchy (system_i/axil_senseedge_0/inst) so that read_saif matches without
// renaming.  Plusargs: +div=<D> +nwin=<windows> +codes=<hex file, 64*nwin 12-bit codes>.
`timescale 1ns/1ps
module pw_inst_wrap (input clk, input aresetn, input [15:0] awaddr, input awvalid, output awready,
  input [31:0] wdata, input [3:0] wstrb, input wvalid, output wready, output [1:0] bresp, output bvalid, input bready,
  input [15:0] araddr, input arvalid, output arready, output [31:0] rdata, output [1:0] rresp, output rvalid, input rready,
  input spi_miso, output spi_sclk, output spi_cs_n, output [3:0] led, input [1:0] sw, output irq);
  axil_senseedge inst (.s_axi_aclk(clk), .s_axi_aresetn(aresetn),
    .s_axi_awaddr(awaddr), .s_axi_awvalid(awvalid), .s_axi_awready(awready),
    .s_axi_wdata(wdata), .s_axi_wstrb(wstrb), .s_axi_wvalid(wvalid), .s_axi_wready(wready),
    .s_axi_bresp(bresp), .s_axi_bvalid(bvalid), .s_axi_bready(bready),
    .s_axi_araddr(araddr), .s_axi_arvalid(arvalid), .s_axi_arready(arready),
    .s_axi_rdata(rdata), .s_axi_rresp(rresp), .s_axi_rvalid(rvalid), .s_axi_rready(rready),
    .spi_miso(spi_miso), .spi_sclk(spi_sclk), .spi_cs_n(spi_cs_n), .led(led), .sw(sw), .irq(irq));
endmodule

module pw_system_wrap (input clk, input aresetn, input [15:0] awaddr, input awvalid, output awready,
  input [31:0] wdata, input [3:0] wstrb, input wvalid, output wready, output [1:0] bresp, output bvalid, input bready,
  input [15:0] araddr, input arvalid, output arready, output [31:0] rdata, output [1:0] rresp, output rvalid, input rready,
  input spi_miso, output spi_sclk, output spi_cs_n, output [3:0] led, input [1:0] sw, output irq);
  pw_inst_wrap axil_senseedge_0 (.clk(clk), .aresetn(aresetn), .awaddr(awaddr), .awvalid(awvalid), .awready(awready),
    .wdata(wdata), .wstrb(wstrb), .wvalid(wvalid), .wready(wready), .bresp(bresp), .bvalid(bvalid), .bready(bready),
    .araddr(araddr), .arvalid(arvalid), .arready(arready), .rdata(rdata), .rresp(rresp), .rvalid(rvalid), .rready(rready),
    .spi_miso(spi_miso), .spi_sclk(spi_sclk), .spi_cs_n(spi_cs_n), .led(led), .sw(sw), .irq(irq));
endmodule

module tb_power;
  reg clk = 0; always #25 clk = ~clk;          // 20 MHz
  reg aresetn = 0;
  reg [15:0] awaddr = 0, araddr = 0; reg awvalid = 0, wvalid = 0, bready = 0, arvalid = 0, rready = 0;
  reg [31:0] wdata = 0; reg [3:0] wstrb = 0;
  wire awready, wready, bvalid, arready, rvalid; wire [1:0] bresp, rresp; wire [31:0] rdata;
  wire spi_sclk, spi_cs_n; wire [3:0] led; wire irq;
  pw_system_wrap system_i (.clk(clk), .aresetn(aresetn), .awaddr(awaddr), .awvalid(awvalid), .awready(awready),
    .wdata(wdata), .wstrb(wstrb), .wvalid(wvalid), .wready(wready), .bresp(bresp), .bvalid(bvalid), .bready(bready),
    .araddr(araddr), .arvalid(arvalid), .arready(arready), .rdata(rdata), .rresp(rresp), .rvalid(rvalid), .rready(rready),
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

  integer i, w, div, nwin, decisions;
  reg [1023:0] codes_file;
  reg [11:0] codes [0:4095];
  reg [7:0]  wimg [0:211];
  initial begin
    if (!$value$plusargs("div=%d", div)) div = 249;
    if (!$value$plusargs("nwin=%d", nwin)) nwin = 16;
    if (!$value$plusargs("codes=%s", codes_file)) codes_file = "codes.hex";
    $readmemh(codes_file, codes);
    $readmemh("weights.hex", wimg);
    repeat (5) @(posedge clk); aresetn = 1; repeat (5) @(posedge clk);
    for (i = 0; i < 64 * nwin; i = i + 1) axi_write(16'h2000 + 4 * i, codes[i]);
    axi_write(16'h1004, 64 * nwin);
    axi_write(16'h1000, 1);
    for (w = 0; w < 53; w = w + 1) axi_write(16'h0020 + 4 * w, {wimg[4*w+3], wimg[4*w+2], wimg[4*w+1], wimg[4*w]});
    axi_write(16'h001C, div);
    axi_write(16'h000C, (3 << 8) | 100);
    axi_write(16'h0018, 3);
    $display("POWER_TB: config done at %0t, running %0d windows at D = %0d", $time, nwin, div);
    axi_write(16'h0000, 1);
    decisions = 0;
    while (decisions < nwin) begin
      axi_read(16'h0018);
      if (rd[0]) begin axi_write(16'h0018, 1); decisions = decisions + 1; end
      else repeat (50) @(posedge clk);
    end
    axi_write(16'h0000, 0);
    axi_read(16'h0008);
    $display("POWER_TB: %0d decisions, last class %0d conf %0d, at %0t", decisions, rd[1:0], rd[9:2], $time);
    $finish;
  end
  initial begin #(64'd4_000_000_000); $display("POWER_TB TIMEOUT"); $finish; end
endmodule

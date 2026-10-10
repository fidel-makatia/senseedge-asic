// Audit harness: nn_engine.v driven through its byte weight port with vectors from files.
// Output per vector: class, confidence, 16 hidden registers (after ReLU), 4 output accumulators.
`timescale 1ns/1ps
module tb_nn_vectors;
  reg clk = 0, rst = 1; always #25 clk = ~clk;
  reg start = 0; wire [2:0] faddr; reg [7:0] fin; wire done; wire [1:0] cls; wire [7:0] conf; wire busy;
  reg wen = 0; reg [7:0] waddr = 0, wdata = 0;
  reg [7:0] fmem [0:7]; always @(*) fin = fmem[faddr];
  nn_engine dut (.clk(clk), .rst(rst), .start(start), .feature_in(fin), .feature_addr(faddr), .done(done),
    .class_id(cls), .confidence(conf), .busy(busy), .wt_wr_en(wen), .wt_wr_addr(waddr), .wt_wr_data(wdata),
    .wt_rd_waddr(6'd0), .wt_rd_wdata());
  reg [7:0] wts [0:211]; reg [7:0] vecs [0:8*8192-1];
  integer i, n, nvec, fd, k;
  initial begin
    $readmemh("weights.hex", wts); $readmemh("vectors.hex", vecs);
    if (!$value$plusargs("nvec=%d", nvec)) nvec = 16;
    repeat (5) @(posedge clk); rst = 0;
    for (i = 0; i < 212; i = i + 1) begin @(posedge clk); wen <= 1; waddr <= i; wdata <= wts[i]; end
    @(posedge clk); wen <= 0;
    fd = $fopen("nn_out.txt", "w");
    for (n = 0; n < nvec; n = n + 1) begin
      for (k = 0; k < 8; k = k + 1) fmem[k] = vecs[n*8 + k];
      @(posedge clk); start <= 1; @(posedge clk); start <= 0;
      @(posedge done); @(posedge clk);
      $fwrite(fd, "%0d %0d", cls, conf);
`ifndef NETLIST
      for (k = 0; k < 16; k = k + 1) $fwrite(fd, " %0d", $signed(dut.hidden[k]));
      for (k = 0; k < 4; k = k + 1)  $fwrite(fd, " %0d", $signed(dut.output_act[k]));
`endif
      $fwrite(fd, "\n");
    end
    $fclose(fd); $finish;
  end
endmodule

// Audit harness: acquisition -> FFT -> features, observed inside the design.
// The ADC is modelled from the MCP3201 data sheet: after CS falls DOUT is high-Z for two
// clocks, a null bit appears on the 2nd falling edge, B11..B0 follow on successive falling
// edges, and the result is then repeated LSB-first (B1, B2, ...).
// Stimulus: codes.hex, 64 twelve-bit codes per window.  Output: chain_out.txt.
`timescale 1ns/1ps
module tb_chain;
  reg clk = 0, rst = 1; always #25 clk = ~clk;                 // 20 MHz
  reg cyc = 0, stb = 0, we = 0; reg [3:0] sel = 0; reg [31:0] dat = 0, adr = 0;
  wire ack; wire [31:0] dato; wire [127:0] la_out; wire [15:0] io_out, io_oeb; wire [2:0] irq;
  reg miso = 1'b0; wire [15:0] io_in = {15'd0, miso};
  senseedge_top dut (.wb_clk_i(clk), .wb_rst_i(rst), .wbs_stb_i(stb), .wbs_cyc_i(cyc), .wbs_we_i(we),
    .wbs_sel_i(sel), .wbs_dat_i(dat), .wbs_adr_i(adr), .wbs_ack_o(ack), .wbs_dat_o(dato),
    .la_data_in(128'd0), .la_data_out(la_out), .la_oenb({128{1'b1}}), .io_in(io_in), .io_out(io_out),
    .io_oeb(io_oeb), .irq(irq));
  wire sclk = io_out[1], csn = io_out[2];

  reg [11:0] code [0:64*1024-1];
  integer sidx = 0, fall = 0; reg [11:0] cur;
  always @(negedge csn) if (!rst) begin cur = code[sidx]; sidx = sidx + 1; fall = 0; miso <= 1'b1; end  // high-Z read as 1
  always @(negedge sclk) if (!csn) begin
    fall = fall + 1;
    if (fall == 2)                    miso <= #5 1'b0;            // null bit
    else if (fall >= 3 && fall <= 14) miso <= #5 cur[14 - fall];  // B11 ... B0
    else if (fall >= 15)              miso <= #5 cur[fall - 14];  // LSB-first repeat: B1, B2
  end

  task wb_write(input [31:0] a, input [31:0] d);
    begin @(posedge clk); cyc <= 1; stb <= 1; we <= 1; sel <= 4'hF; adr <= a; dat <= d;
      @(posedge clk); wait (ack === 1'b1); @(posedge clk); cyc <= 0; stb <= 0; we <= 0; repeat (2) @(posedge clk); end
  endtask

  // cycles spent in every FSM state during the first window (buffer full -> decision)
  integer n_fft [0:7], n_fe [0:7], n_nn [0:7]; reg counting = 0; integer t_cyc = 0, t_valid = 0;
  always @(posedge clk) begin
    t_cyc = t_cyc + 1;
    if (counting) begin
      n_fft[dut.u_fft.state] = n_fft[dut.u_fft.state] + 1;
      n_fe[dut.u_feature.state] = n_fe[dut.u_feature.state] + 1;
      n_nn[dut.u_nn.state] = n_nn[dut.u_nn.state] + 1;
    end
  end

  integer nwin, w, i, fd, div, t_en;
  initial begin
    $readmemh("codes.hex", code);
    if (!$value$plusargs("nwin=%d", nwin)) nwin = 1;
    if (!$value$plusargs("div=%d", div)) div = 4;
    for (i = 0; i < 8; i = i + 1) begin n_fft[i] = 0; n_fe[i] = 0; n_nn[i] = 0; end
    fd = $fopen("chain_out.txt", "w");
    repeat (10) @(posedge clk); rst = 0; repeat (5) @(posedge clk);
    wb_write(32'h3000_001C, div);
    wb_write(32'h3000_0000, 32'd1); t_en = t_cyc;
    for (w = 0; w < nwin; w = w + 1) begin
      @(posedge dut.samples_valid); #1;
      $fwrite(fd, "T %0d %0d\n", w, t_cyc - t_en);          // clock count at each full buffer
      if (w == 0) begin counting = 1; t_valid = t_cyc; $fwrite(fd, "ACQ %0d\n", t_cyc - t_en); end
      $fwrite(fd, "S %0d", w);
      for (i = 0; i < 64; i = i + 1) $fwrite(fd, " %0d", $signed(dut.u_spi_adc.sample_buf[i]));
      $fwrite(fd, "\n");
      @(posedge dut.nn_done); #1;
      if (w == 0) begin
        counting = 0;
        $fwrite(fd, "LAT %0d\n", t_cyc - t_valid);
        $fwrite(fd, "FFT"); for (i = 0; i < 6; i = i + 1) $fwrite(fd, " %0d", n_fft[i]); $fwrite(fd, "\n");
        $fwrite(fd, "FE");  for (i = 0; i < 5; i = i + 1) $fwrite(fd, " %0d", n_fe[i]);  $fwrite(fd, "\n");
        $fwrite(fd, "NN");  for (i = 0; i < 7; i = i + 1) $fwrite(fd, " %0d", n_nn[i]);  $fwrite(fd, "\n");
      end
      $fwrite(fd, "M %0d", w);
      for (i = 0; i < 32; i = i + 1) $fwrite(fd, " %0d", dut.u_fft.mag_buf[i]);
      $fwrite(fd, "\nF %0d", w);
      for (i = 0; i < 8; i = i + 1) $fwrite(fd, " %0d", dut.u_feature.features[i]);
      $fwrite(fd, "\n");
    end
    $fclose(fd); $finish;
  end
endmodule

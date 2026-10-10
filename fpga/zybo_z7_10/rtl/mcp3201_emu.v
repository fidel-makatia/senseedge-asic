// SPDX-License-Identifier: Apache-2.0
// MCP3201 emulator for the FPGA deployment: plays 12-bit codes from a RAM filled over the
// bus, with the data-sheet frame timing used by the audit harness (verilog/dv/audit):
// after CS falls DOUT is high (high-Z read as 1); a null bit on the 2nd falling SCLK edge;
// B11..B0 on falling edges 3..14; the LSB-first repeat B1, B2, ... from edge 15.
// Same clock domain as the capture unit; MISO changes one clock after each SCLK falling
// edge, which the capture unit (sampling at the edge) sees as the previous bit, exactly as
// the simulation model with its #5 delay.
`default_nettype none
module mcp3201_emu #(parameter AW = 12) (
    input  wire          clk,
    input  wire          rst,
    input  wire          sclk,
    input  wire          cs_n,
    output reg           miso,
    input  wire          wr_en,
    input  wire [AW-1:0] wr_addr,
    input  wire [11:0]   wr_data,
    input  wire [AW:0]   len,          // codes to play before wrapping to the first
    output wire [AW:0]   play_idx      // next code index (for debug read-back)
);
    reg [11:0] mem [0:(1<<AW)-1];
    always @(posedge clk) if (wr_en) mem[wr_addr] <= wr_data;

    reg          sclk_d, cs_d;
    reg [11:0]   cur;
    reg [AW:0]   idx;
    reg [4:0]    fall;
    wire [4:0]   f = fall + 5'd1;     // edge count including this one
    assign play_idx = idx;

    always @(posedge clk) begin
        if (rst) begin
            sclk_d <= 1'b0; cs_d <= 1'b1; cur <= 12'd0; idx <= {(AW+1){1'b0}};
            fall <= 5'd0; miso <= 1'b1;
        end else begin
            sclk_d <= sclk;
            cs_d   <= cs_n;
            if (cs_d && !cs_n) begin                       // frame start
                cur  <= mem[idx[AW-1:0]];
                idx  <= (idx + 1'b1 >= len) ? {(AW+1){1'b0}} : idx + 1'b1;
                fall <= 5'd0;
                miso <= 1'b1;
            end else if (!cs_n && sclk_d && !sclk) begin   // SCLK falling edge
                fall <= f;
                if (f == 5'd2)                     miso <= 1'b0;            // null bit
                else if (f >= 5'd3 && f <= 5'd14)  miso <= cur[5'd14 - f];  // B11 .. B0
                else if (f >= 5'd15 && f <= 5'd25) miso <= cur[f - 5'd14];  // B1, B2, ...
            end
        end
    end
endmodule
`default_nettype wire

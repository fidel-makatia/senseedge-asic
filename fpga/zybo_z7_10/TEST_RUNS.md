# Test runs on the Zybo Z7-10 (Rev. B)

These are the runs that fill the Rev. B slots of the TVLSI manuscript (Section IX-C/D, the
last row of Table VI, the "Rev. B, FPGA" column of Table VII, and ladder step (i') of
Section VIII-B). Run them in this order; each one assumes the previous passed. `README.md`
has the wiring, the build and the Vitis steps.

| # | Run | How | Expected / record | Fills |
|---|---|---|---|---|
| 1 | Build | `vivado -mode batch -source scripts/build.tcl` on a Linux or Windows machine with Vivado ≥ 2022.1 | `build/senseedge_zybo.bit`, `.xsa`, `utilization.txt`, `timing.txt` (WNS ≥ 0 at 20 MHz); also run `report_power` and keep it. Yosys estimate to compare against: 9.6 k LUT, 6.1 k FF, 3 RAMB18, 7 DSP | IX-C utilization, timing and power estimate; Table VII "Power" |
| 2 | Self-test | SW0 down, *Run As > Launch on Hardware*, terminal 115200 8N1 | `wrapper id 0x5e5e0001`, `weights: 53 words written, 0 read-back errors`, `self-test: 32 bins, 8 features, class: PASS, identical to the RTL audit values`, `*** SELF-TEST PASSED ***`, then `CLASS:BEARING_WEAR CONF:255` lines; LD0 after four fault decisions, LD2 on, LD3 toggling | Table VI last row; first sentence of IX-D |
| 3 | Live ADC | SW0 up; node accelerometer + MCP3201 on Pmod JE (JE1 MISO, JE2 SCLK, JE3 CS, 3V3, GND); shipped weights from `firmware/nn_weights.h`; divider 249 (2.29 kS/s) | `CLASS:... CONF:... ALARM:...` once per window (about every 28 ms); tap the sensor and watch the class and confidence move | Evidence the real front end works behind the same board; no paper number by itself |
| 4 | Register-map update | With the emulator replaying a fixed record, write a second weight set over `SE_BASE+0x20..0xF0`, read it back, observe the class flip; write the first set back | The decision changes with the weights alone, read-back 0 errors both times | VIII-D "model update", IX-D |
| 5 | Physical-fault replay (ladder step i') | For each recorded session of group F (ADC codes from the node), push 64-window blocks into the emulator RAM (`0x2000`, 4096 codes), set `EMU_LEN`, run, log class and confidence per window; weights trained with `ml/train_corrected_exact.py` on the corrected arithmetic | Accuracy per session, session-bootstrap 95 % interval, confusion matrix; compare with ladder step (i) on Rev. A | VIII-B (i'), IX-D, Table VII "Accuracy" |
| 6 | CWRU replay (optional) | As 5 with the CWRU records resampled to the chip rate | A reproducible number next to F7 | VIII-E |

Register map of the wrapper (AXI-Lite at `0x43C00000`): `0x0000–0x00FF` SenseEdge registers as
in the chip (`0x20–0xF0` the 53 weight words); `0x1000` EMU_CTRL (bit 0 selects the emulator);
`0x1004` EMU_LEN; `0x1008` ID = `0x5E5E0001`; `0x100C` `{emu_idx[7:0], sw[1:0]}`;
`0x2000–0x5FFF` emulator code RAM, one 12-bit code per word.

## Still to write (after run 2 passes)

- A record loader: host script (`sim/` already has the record format) → UART → PS → emulator
  RAM, so that runs 5 and 6 do not need rebuilding the program per session. An SD-card loader is
  the alternative if the UART is too slow for 210 sessions.
- A second weight set and a UART command in `sw/senseedge_zybo.c` for run 4.
- A host-side logger that writes `session, window, class, confidence` CSV for runs 5 and 6.

## What the FPGA does not provide

No silicon power or energy figure: the Vivado estimate goes only into the like-for-like FPGA
column of the comparison table. The evidence that a SKY130 mapping of Rev. B computes the same
values is the gate-level rows of Table VI (`verilog/dv/audit/run_gl.py`), not the FPGA.

## Status

| Run | Status (10 Oct 2026) |
|---|---|
| 1 | not run: no Vivado on the authoring machine (macOS); TCL untested |
| 2 | wrapper sequence verified in Icarus (`sim/run_sim.sh`, PASS) with the values the program expects |
| 3–6 | pending the board and, for 5–6, the group F recordings |

# Test runs on the Zybo Z7-10 (Rev. B)

These runs fill the Rev. B slots of the TVLSI manuscript (Section IX-C/D, the last row of
Table VI, the "Rev. B, FPGA" column of Table VII, and the dataset results of Section VIII).
No sensor or motor is needed: runs 3 to 7 replay public datasets through the on-chip ADC
emulator (`host/README.md`). Run them in this order; each one assumes the previous passed.
`README.md` has the wiring, the build and the Vitis steps.

| # | Run | How | Expected / record | Fills |
|---|---|---|---|---|
| 1 | Build | `vivado -mode batch -source scripts/build.tcl` on a Linux or Windows machine with Vivado >= 2022.1 | `build/senseedge_zybo.bit`, `.xsa`, `utilization.txt`, `timing.txt` (WNS >= 0 at 20 MHz); also `report_power`. Yosys estimate to compare against: 9.6 k LUT, 6.1 k FF, 3 RAMB18, 7 DSP | IX-C utilization, timing, power estimate; Table VII "Power" |
| 2 | Self-test | SW0 down, *Run As > Launch on Hardware*, terminal 115200 8N1 | `wrapper id 0x5e5e0002, protocol 1`, `weights: 53 words written, 0 read-back errors`, `self-test: 32 bins, 8 features, class: PASS, identical to the RTL audit values`, `*** SELF-TEST PASSED ***`, then `CLASS:BEARING_WEAR CONF:255` lines; LD0 after four fault decisions, LD2 on, LD3 toggling | Table VI last row; first sentence of IX-D |
| 3 | Protocol | close the terminal; `python3 host/test_protocol_mock.py --port <USB-UART>` (same checks as against the mock) | 11 PASS lines | — (prerequisite) |
| 4 | CWRU replay, three sample rates | `host/replay.py` with `records/cwru_d249_g512.npz` + `ml/records/cwru_d249_g512_weights.npz`, then `cwru_d142_g512` / `cwru_d70_g512` with their weights; `--split test` (unseen 3 hp load) | every window bit-exact (features, class, confidence, alarm; bins on every 20th block); accuracy equal to `ml/records/cwru_*_train.md`: 75.9 / 65.1 / 80.5 % window, 90 / 80 / 100 % sequence | VIII sample-rate policy; Table VIII |
| 5 | MaFaulDa replay | `host/replay.py --records records/mafaulda_d249.npz --weights ml/records/mafaulda_d249_weights.npz` (about 25 min at 115200 baud); then `mafaulda_d70` and `mafaulda_tracked` | bit-exact as above; accuracy, CI, confusion matrix, per-condition table equal to `mafaulda_*_train.md` | VIII main result; IX-D; Table VIII |
| 6 | Divider independence | `host/divider_check.py --records records/cwru_d249_g512.npz --weights ml/records/cwru_d249_g512_weights.npz` (D = 249, 9, 4 on the same 32 windows) | identical bins/features/decisions; time per window 28.0 / 1.12 / 0.56 ms plus transfer | IX-C |
| 7 | Register-map update | `host/weight_update.py` on a BEARING_WEAR test sequence | set A: fault decisions, alarm 1; set B (outputs rotated): decisions moved as the model predicts, alarm 0; read-back 0 errors each time | VIII-D "model update", IX-D |
| 8 | Live ADC (optional, needs the node's sensor) | SW0 up; accelerometer + MCP3201 on Pmod JE (JE1 MISO, JE2 SCLK, JE3 CS, 3V3, GND) | `CLASS:... CONF:... ALARM:...` once per window | evidence only |

Wrapper register map (AXI-Lite at `0x43C00000`): `0x0000–0x00FF` the chip's registers
(`0x20–0xF0` the 53 weight words); `0x1000` EMU_CTRL (bit 0 emulator, bit 1 soft reset of chip
+ emulator); `0x1004` EMU_LEN; `0x1008` ID = `0x5E5E0002`; `0x100C` `{emu_idx[7:0], sw[1:0]}`;
`0x2000–0x5FFF` emulator code RAM, one 12-bit code per word.

## Verified before the board (11 Oct 2026)

| Check | Result |
|---|---|
| `ref_models.fft64`/`chain` vs `fft_engine.v` in Icarus: 1024 random, tone and corner windows; 1024 CWRU windows | 2048/2048 bit-exact (32 bins + 8 features each) |
| `sim/run_replay_check.py`: 256 tone windows and 256 CWRU windows through the wrapper RTL in 4 blocks of 64 with soft reset + weight reload per block | PASS: 0 mismatches in bins, features, class, confidence, alarm; 0 read-back errors |
| `host/test_protocol_mock.py`: the firmware's protocol handler compiled against the mock chip | 11/11 PASS |
| `ml/train_on_records.py`: kept weights vs `nn_engine.v` on 400 test vectors | 400/400 bit-exact |
| `sim/run_sim.sh`: the self-test sequence | PASS |

## What the FPGA does not provide

No silicon power or energy figure: the Vivado estimate goes only into the like-for-like FPGA
column of the comparison table. The evidence that a SKY130 mapping of Rev. B computes the same
values is the gate-level rows of Table VI (`verilog/dv/audit/run_gl.py`), not the FPGA.

## Status

| Run | Status (11 Oct 2026) |
|---|---|
| 1 | not run: no Vivado on the authoring machine (macOS); TCL untested |
| 2 | sequence verified in Icarus (`sim/run_sim.sh`, PASS) |
| 3–7 | host side and firmware verified in simulation and against the mock; pending the board |
| 8 | pending a sensor |

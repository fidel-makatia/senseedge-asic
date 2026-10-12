# Test runs on the Zybo Z7-10 (Rev. B)

These runs fill the Rev. B slots of the TVLSI manuscript (Section IX-C/D, the last row of
Table VII, the "Rev. B, FPGA" column of Table VIII, and the dataset results of Section VIII).
No sensor or motor is needed: runs 3 to 7 replay public datasets through the on-chip ADC
emulator (`host/README.md`). Run them in this order; each one assumes the previous passed.
`README.md` has the wiring, the build and the Vitis steps.

| # | Run | How | Expected / record | Fills |
|---|---|---|---|---|
| 1 | Build | `vivado -mode batch -source scripts/build.tcl` on a Linux or Windows machine with Vivado >= 2022.1 | `build/senseedge_zybo.bit`, `.xsa`, `utilization.txt`, `timing.txt` (WNS >= 0 at 20 MHz); also `report_power`. Yosys estimate to compare against: 9.6 k LUT, 6.1 k FF, 3 RAMB18, 7 DSP | IX-C utilization, timing, power estimate; Table VIII "Power" |
| 2 | Self-test | SW0 down, *Run As > Launch on Hardware*, terminal 115200 8N1 | `wrapper id 0x5e5e0002, protocol 1`, `weights: 53 words written, 0 read-back errors`, `self-test: 32 bins, 8 features, class: PASS, identical to the RTL audit values`, `*** SELF-TEST PASSED ***`, then `CLASS:BEARING_WEAR CONF:255` lines; LD0 after four fault decisions, LD2 on, LD3 toggling | Table VII last row; first sentence of IX-D |
| 3 | Protocol | close the terminal; `python3 host/test_protocol_mock.py --port <USB-UART>` (same checks as against the mock) | 11 PASS lines | — (prerequisite) |
| 4 | CWRU replay, three sample rates | `host/replay.py` with `records/cwru_d249_g512.npz` + `ml/records/cwru_d249_g512_weights.npz`, then `cwru_d142_g512` / `cwru_d70_g512` with their weights; `--split test` (unseen 3 hp load) | every window bit-exact (features, class, confidence, alarm; bins on every 20th block); accuracy equal to `ml/records/cwru_*_train.md`: 75.9 / 65.1 / 80.5 % window, 90 / 80 / 100 % sequence | VIII sample-rate policy; Table VI |
| 5 | MaFaulDa replay | `host/replay.py --records records/mafaulda_d249.npz --weights ml/records/mafaulda_d249_weights.npz` (about 25 min at 115200 baud); then `mafaulda_d70` (about 90 min) and `mafaulda_tracked` (about 13 min) | bit-exact as above; window / sequence accuracy equal to `mafaulda_*_train.md`: 71.7 / 76.6 % (d249), 63.9 / 71.7 % (d70), 67.6 / 67.8 % (tracked); confusion matrix and per-condition table likewise | VIII main result; IX-D; Table VI |
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
| `ref_models.fft64`/`chain` vs `fft_engine.v` in Icarus: 1024 random, tone and corner windows; 1024 CWRU windows; 1024 MaFaulDa windows | 3072/3072 bit-exact (32 bins + 8 features each) |
| `sim/run_replay_check.py`: 256 tone, 256 CWRU and 256 MaFaulDa windows (with the kept weights) through the wrapper RTL in 4 blocks of 64 with soft reset + weight reload per block | PASS each: 0 mismatches in bins, features, class, confidence, alarm; 0 read-back errors |
| `host/test_protocol_mock.py`: the firmware's protocol handler compiled against the mock chip | 11/11 PASS |
| `ml/train_on_records.py`: the six kept weight sets (three CWRU, three MaFaulDa) vs `nn_engine.v` on 400 test vectors each | 6 x 400/400 bit-exact |
| `sim/run_sim.sh`: the self-test sequence | PASS |

## Reproduced on the board machine (11 Oct 2026, Windows 11, Icarus 12.0)

| Check | Result |
|---|---|
| `verilog/dv/audit/run_audit.py` (experiments A–E) | ALL PASS: 2400/2400 NN vectors, 4096/4096 ADC codes, 32/32 tones, 53/53 weight words, 8/8 bus windows, 728 clocks buffer-full to decision |
| `check_fft_model.py --records cwru_d249_g512.npz` | 1024/1024 windows bit-exact (32 bins + 8 features) |
| Board at 921600 baud (`SE_BAUD=921600 vitis -s scripts/vitis_build.py`) | self-test spectrum PASS; CWRU D = 249 replay 3,608/3,608 bit-exact at **164 windows/s** (65 at 115200) |

Two things to know when repeating these: (1) never run two of the audit scripts at once — they
share `verilog/dv/audit/build/` and the stimulus file of one overwrites the other's (the
truncated outputs then look like RTL mismatches); (2) the Vitis workspace must sit at a short
path (`scripts/vitis_build.py` uses `%USERPROFILE%\se_ws`) or the BSP build fails on Windows'
250-character object-path limit.

## What the FPGA does not provide

No silicon power or energy figure: the Vivado estimate goes only into the like-for-like FPGA
column of the comparison table. The evidence that a SKY130 mapping of Rev. B computes the same
values is the gate-level rows of Table VII (`verilog/dv/audit/run_gl.py`), not the FPGA.

## Status

Runs 1–7 were done on the board on 11 Oct 2026 (Vivado/Vitis 2026.1 on Windows, Zybo Z7-10 on
COM10, 115200 baud). Every file named below is under `host/results/`; the figures and LaTeX
tables are written by `host/paper_figures.py` and `host/paper_tables.py` into
`host/results/figures/` and `host/results/tables/`. `host/run_board_tests.py --port COM10`
repeats runs 2–7 in one command.

| Run | Result (11 Oct 2026) |
|---|---|
| 1 | `scripts/build_zybo.tcl` (installs the Digilent board files from the AMD store, then `build.tcl`): bitstream in 6 min. Whole design 11,200 LUT (63.6 %), 6,317 FF, 1.5 BRAM, 6 DSP; the macro alone 10,718 LUT / 5,640 FF / 6 DSP (FFT 8,762 LUT, NN 1,366). WNS +8.38 ns, WHS +0.04 ns at 20 MHz. `report_power`: 27 mW for the PL wrapper, 1.40 W for the PS7. `build/utilization*.txt`, `timing.txt`, `power.txt` |
| 2 | UART: `wrapper id 0x5E5E0002, protocol 1`, `weights: 53 words written, 0 read-back errors`, `self-test: ... PASS, identical to the RTL audit values`, `*** SELF-TEST PASSED ***`, then `CLASS:BEARING_WEAR CONF:255` with the alarm on the fourth fault decision (`selftest_uart.txt`). Over the protocol: 32/32 bins, 8/8 features, class 1 / 255 equal to `ref_models.chain` (`selftest_spectrum.json`) |
| 3 | not applicable to the board: `test_protocol_mock.py` forces `--mock` and checks the mock chip's arithmetic. The protocol is exercised on the board by run 2 (identify, weight round trip, record, run, read-back) and by every block of runs 4–7 |
| 4 | CWRU, 3 hp held out: 2.29 kS/s **75.86 %** window (CI 57.5–88.4), 77.7 % balanced, 90 % sequence; 4.0 kS/s **65.07 %** (47.0–80.7), 60.9 %, 80 %; 8.0 kS/s **80.54 %** (72.3–88.1), 83.7 %, 100 %. Confusion matrices identical to the host model. Bit-exact: 3,608 + 6,324 + 12,764 = **22,696 windows, 0 mismatches** in features, class, confidence and alarm; bins checked on 1,149 windows, 0 mismatches. 65 windows/s (`fpga_cwru_d*_g512.{csv,json,md}`) |
| 5 | MaFaulDa (12.9 GB fetched and extracted here; records regenerated with the same gains as the committed ones; the committed weights): 2.29 kS/s **71.65 %** window (CI 69.0–74.1), 73.6 % balanced, 76.6 % sequence majority; 8.0 kS/s **63.87 %** (61.4–66.1), 66.5 %, 71.7 %; tracked f_s = 32 f_rot **67.56 %** (64.2–70.6), 73.5 %, 67.8 %. Confusion matrices identical to the host model. Bit-exact: 103,840 + 369,340 + 53,066 = **526,246 windows, 0 mismatches**; bins checked on 26,688 windows, 0 mismatches. Rates: 132 windows/s (D = 249), 218 (D = 70), 17 (tracked: each sequence replayed at its own real divider) (`fpga_mafaulda_{d249,d70,tracked}.{csv,json,md}`) |
| 6 | D = 4, 9, 249 on the same 32 windows: identical bins, features and decisions; 7.51 / 8.01 / 35.01 ms per window incl. transfer against 0.56 / 1.12 / 28.00 ms capture time (`divider_check.json`) |
| 7 | IR007 test sequence, 16 windows: set A → 14× class 1, 2× class 3, alarm from window 3; set B → 14× class 0, 2× class 2, no alarm; set A again → identical to the first run; 0 read-back errors each load; 16 of 16 decisions moved by the register-map write alone (`weight_update.json`) |
| 8 | pending a sensor |
| 9 | Streaming (`host/stream_demo.py`, replayed at the deployment divider so the time axis is machine time): a healthy sequence followed by a fault sequence; per window the chip's 32 bins, 8 features, decision, confidence and the hardware alarm. CWRU normal → IR007 at 2.29 kS/s: hardware alarm **4 windows = 112 ms** after the onset (N = 3), 312/359 healthy and 363/363 fault windows decided correctly over 20 s; alarm depth N = 0..7 in hardware: delay N+1 windows, false alarms in 6/6, 2/6, then 0/6 healthy blocks; 914/914 windows bit-exact. CWRU normal → OR007 at 8.0 kS/s: alarm **32 ms** after the onset, 0/20 healthy blocks with a false alarm from N = 2; 2,744/2,744 bit-exact. MaFaulDa at 2.29 kS/s, normal → imbalance/35g: alarm 4 windows (112 ms) after the onset; normal → underhang/outer_race/35g: 6 windows; the healthy class of MaFaulDa is harder (72 % recall), so false alarms fall with N instead of vanishing: 42/42, 36/42, 24/42, 13/42, 6/42, 5/42, 1/42, 1/42 healthy blocks at N = 0..7; 544/544 bit-exact each. Figures `fig_<name>_onset`, `_timeline`, `_alarm_depth`, `_class_spectra` for `stream_cwru_d249`, `stream_cwru_d70`, `stream_mafaulda_d249_imbalance`, `stream_mafaulda_d249_bearing` |
| 10 | Severity ladder (`host/severity_stream.py`): healthy, then imbalance 6 → 10 → 15 → 20 → 25 → 30 → 35 g, one speed-matched test sequence (≈ 50 Hz) per step, 32 windows each. The chip's band 1–4 feature rises monotonically with the mass: 22 → 43 → 83 → 132 → 153 → 191 → 218 → 252 at 2.29 kS/s (19 → 37 → 73 → 109 → 128 → 159 → 183 → 215 tracked), with 100 % fault decisions from 6 g on; 256/256 windows bit-exact in each run. Without speed matching the ladder is confounded (imbalance force ∝ speed²; at 14–23 Hz the 1× line falls in bin 0, which no band counts). `fig_mafaulda_imbalance_severity`, `fig_mafaulda_imbalance_tracked_severity` |
| 11 | Window period and throughput (`host/throughput_check.py`, firmware command `T`, Cortex-A9 global timer at 333.3 MHz): over 63 windows after the first decision, **2303.9 clocks at D = 0 and exactly 2240 (D+1) clocks (±0.000 %) at D = 1, 4, 9, 70, 249** — the two values predicted in the test plan (2304 at D = 0; 2240(D+1)); peak **8,681 classifications/s** at D = 0, 35.7/s at the reset divider. `results/throughput.json` |
| 12 | Power. The Zybo has no current sense and no meter was on the bench, so power is estimated, not measured: `report_power` on the implemented design gives 27 mW for the PL wrapper (vectorless, 1.40 W for the Zynq PS); an RTL-level SAIF from xsim matched only 2 % of the implemented nets, so the activity-annotated figure comes from simulating the **implemented netlist** (`write_verilog -mode funcsim`, `sim/tb_power.v`, `scripts/power_saif.tcl`) with recorded vibration windows at D = 249 and D = 4 — see `build/power/power_saif_d*.txt` and `fig_power_blocks`. A measured number needs a shunt on the PL rail or a Rev. A die on the bench (test plan C2–C5) |

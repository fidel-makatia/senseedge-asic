# Silicon test plan for SenseEdge Rev. A

`test_plan.tex` / `test_plan.pdf` is the complete pre-silicon test plan (written 2 Oct 2026,
before first silicon; packaged parts from the ChipFoundry CI2605 shuttle are expected late 2026).
It holds the procedures, the instrument list (I1–I7), the golden vector sets (V1–V5), the
acceptance criteria, the predictions fixed before silicon, the reporting rules, and the risks
specific to this chip. Build the PDF with `tectonic test_plan.tex` or any pdfLaTeX.

Decision G0 was taken on 9–10 Oct 2026: the paper is framed around what Rev. A (the fabricated
part) measures, and the five audit defects are corrected in Rev. B, the RTL on this branch, which
is validated on gate-level netlists, in Caravel co-simulation and on a Zybo Z7-10 FPGA
(`../../fpga/zybo_z7_10/`). No second tape-out is planned. The plan's section "Revision B, if the
journal version is to claim the on-chip classifier" is therefore historical.

## Groups

| Group | IDs | Dice | What it establishes | Paper slot (TVLSI manuscript) |
|---|---|---|---|---|
| A, bring-up | A1 continuity, A2 quiescent rails, A3 housekeeping ID, A4 boot, A5 pad handover, A6 register map, A7 first bin-8 window | all | The part is alive and the first window is bit-exact to the RTL (**gate G1**) | VII-A setup, VII-B |
| B, functional | B1 capture transfer, B2 FFT (≥10^5 windows), B3 features and band map, B4 timing constants, B5 alarm and interrupt, B6 soak (10^8 windows), N1 network as fabricated | 3 in full, all on V2 | Zero mismatches against the netlist-derived reference; the 729-clock and 2240(D+1) constants; the 22-of-212 weight defect documented on silicon (**gate G2** = B2, B3 clean) | VII-B, VII-G, Table IV rows 1–5 |
| C, electrical | C1 leakage, C2 power vs frequency, C3 shmoo, C4 minimum-energy point, C5 energy per classification, C6 phase-resolved current, C7 interface floor, C8 temperature, C9 twenty dice | 3 at four temperatures; ≥20 at 25 °C | Energy per clock, per window at both clock policies, f_max, V_min, die-to-die statistics (**gate G3** = Section VII complete) | VII-C, VII-D, VII-E, Table IV rows 6–12, Table VII measured column |
| D, real sensor | D1 ADC and capture, D2 shaker response, D3 feature fidelity, D4 offset and orientation | node | Chain response in physical units; DC-offset leakage into the high band and its removal | VII-A (as-built node), VIII-A |
| E, baselines | E1 same-die software pipeline, E2 commercial MCU, E3 node power, E4 firmware-classifier cost | 1 | Hardware vs software on the same die; the one measured row of the comparison table; cost of the hybrid decision on Rev. A | VII-F, Table VII MCU row |
| F, physical faults | F1 testbed, F2 ~210 sessions, F3 model ladder, F4 sample-rate policy, F5 two-week live run, F6 second machine, F7 CWRU replay | node | Accuracy with session-bootstrap intervals, detection delay, false alarms vs alarm depth, retargeting cost (**gate G4** = Section VIII complete) | VIII-A to VIII-E; ladder step (i') runs the same sessions on Rev. B (see `../../fpga/zybo_z7_10/TEST_RUNS.md`) |

Predictions to correlate are in the plan's Section "Pre-silicon predictions" and in Table IV of
the manuscript: 729 clocks buffer-full to decision; 2240(D+1) clocks per window (2304 at D=0);
capture transfer 2(c−2048)+c1 with wrap at ±1024; bands 2–5, 6–11, 12–21, 22–31; 22 of 212
weights reachable; 12.9 mW at 20 MHz, 1.8 V, 25 °C; 0.65 nJ per clock; 0.7 µW leakage; 1.5 µJ
per window at D=0; 0.36 mJ per window at reset; f_max ≈ 42 MHz; hold margin +0.06 ns fast corner.

## Minimum set for the journal submission

If time is short: A (all parts), B1–B4, N1, C1–C5, C9, E1, E4, F1–F3, F5. C8, D, E2, E3, F4,
F6 and F7 strengthen the paper but are not what reviewers reject over.

## Rules that apply to every test

- The reference for every bit-exact check is the integer model derived from the netlist
  (`../../verilog/dv/audit/ref_models.py` for Rev. B; the Rev. A models are the
  `proposal/src` scripts of the paper package), never the floating-point specification.
- Every register access goes through the management core, with the address bits it can drive.
- Record the raw data of every sweep with the die number, supply, frequency, temperature and
  firmware hash; the session, not the window, is the statistical unit in group F.
- A Wishbone access to the macro below its working voltage never returns: the host must time
  out and pulse RESETB.

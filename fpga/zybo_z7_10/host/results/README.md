# Measured results on the Zybo Z7-10 (Rev. B RTL), 11 Oct 2026

Everything in this directory was measured on the board (Vivado/Vitis 2026.1, xc7z010clg400-1,
20 MHz, program `sw/senseedge_zybo.c` at 921600 baud) except the `host_*` files, which are
the Python reference chain's predictions and exist only to show that the board reproduces
them bit for bit. Figures are vector PDF (IEEE column widths: 3.5 in single, 7.16 in double)
with a 600-dpi PNG twin; tables are booktabs LaTeX. Regenerate with `paper_figures.py`
and `paper_tables.py`; re-measure with `run_board_tests.py`, `run_mafaulda.py`, `stream_demo.py`.

## Figures

| File | Width | What it shows | Data |
|---|---|---|---|
| `fig_cwru_accuracy` | single | Window / balanced / sequence-majority accuracy on the held-out 3 hp load at 2.29, 4.0 and 8.0 kS/s; whiskers = 95 % sequence-bootstrap interval on the window accuracy | `fpga_cwru_d{249,142,70}_g512.json` |
| `fig_cwru_confusion` | double | Row-normalised confusion matrices with counts, one panel per sample rate | same |
| `fig_cwru_per_condition` | single | Accuracy of each held-out CWRU file (fault type x severity) at the three rates; shows the IR014 collapse at low rates | same |
| `fig_mafaulda_accuracy`, `_confusion`, `_per_condition` | | The same three for MaFaulDa (2.29 kS/s, 8.0 kS/s, tracked f_s = 32 f_rot; 42 conditions) | `fpga_mafaulda_{d249,d70,tracked}.json` |
| `fig_cwru_d249_onset` | double | **Deployment figure.** 24 healthy windows then an inner-race fault streamed at 2.29 kS/s: ADC signal, the chip's own 32-bin spectrogram (read back from FFT_DATA), the four band-energy features and the confidence, the per-window decision and the hardware alarm (112 ms after the onset at N = 3) | `stream_cwru_d249.json` |
| `fig_cwru_d70_onset` | double | The same at 8.0 kS/s with an outer-race fault (alarm 32 ms after the onset) | `stream_cwru_d70.json` |
| `fig_cwru_d249_timeline`, `fig_cwru_d70_timeline` | double | 20 s of machine time: whole healthy sequence then whole fault sequence; RMS, chip spectrogram, decisions; alarm from the alarm model over the chip's decisions (the replay resets the chip every 64 windows) | same |
| `fig_cwru_d249_alarm_depth`, `fig_cwru_d70_alarm_depth` | single | Alarm depth N = 0..7 **in hardware**: detection delay after the onset (N+1 windows) and the fraction of healthy blocks with a false alarm (6/6 at N = 0, 0 from N = 2) | same |
| `fig_cwru_d249_class_spectra`, `fig_cwru_d70_class_spectra` | double | One test window per class: the 64 ADC codes and the chip's 32-bin spectrum, with the chip's decision and confidence | same |
| `fig_mafaulda_imbalance_severity`, `fig_mafaulda_imbalance_tracked_severity` | double | **Progression figure.** Healthy, then imbalance growing 6 → 35 g at one shaft speed: the chip's band 1-4 and total-energy features, confidence, fault fraction and decisions per step — the trend a maintenance system would threshold | `severity_mafaulda_imbalance*.json` |
| `fig_mafaulda_d249_{imbalance,bearing}_onset`, `_timeline`, `_alarm_depth`, `_class_spectra` | | The streaming set on MaFaulDa (healthy → 35 g imbalance; healthy → outer-race bearing fault) | `stream_mafaulda_d249_*.json` |
| `fig_power_blocks` | single | Vivado vectorless power estimate of the wrapper per block (FFT 21 mW, NN 4 mW, rest < 1 mW each; 27 mW total) — an estimate, the Zybo has no current sense | `results/power_vectorless_hier.txt` |
| `fig_divider` | single | Time per window at D = 4, 9, 249 against 64 x 35 (D+1) / f_clk; outputs identical at every D | `divider_check.json` |
| `fig_weight_update` | single | Register-map model update: 16 windows under weight set A, set B (outputs rotated) and A again; decisions and alarm | `weight_update.json` |
| `fig_selftest_spectrum` | single | The bin-8 self-test record: 32 bins from the board over the reference (identical) | `selftest_spectrum.json` |
| `fig_block_design` | | Vivado block design (PS7, AXI interconnect, `axil_senseedge` wrapper) | `build/vivado` |

`paper_figures_compact.py` writes the shortened/compact variants used in the manuscript for the page
budget: `fig_<stream>_onset_short` (4 panels, 3.9 in), `fig_<stream>_onset_compact` (codes, bins,
decisions; 2.7 in) and `fig_<name>_severity_compact` (features with the fault share per step,
decisions; 2.3 in).

## Tables (`tables/`)

| File | Content |
|---|---|
| `tab_cwru_bitexact.tex`, `tab_mafaulda_bitexact.tex` | windows, blocks, and matches for features, class+confidence, bins (every 20th block) and alarm |
| `tab_cwru_accuracy.tex`, `tab_mafaulda_accuracy.tex` | window accuracy with 95 % CI, balanced, sequence majority, per-class recall |
| `tab_stream.tex` | streaming runs: healthy/fault windows decided correctly, delay at N = 3, false-alarm blocks at N = 0/1/2, bit-exactness |
| `tab_divider.tex` | divider independence timings |
| `tab_fpga.tex` | utilization, timing and power of the Zybo implementation; per-block LUT/FF/DSP of the macro |
| `tab_metrics.tex` | one-table summary: resources, slack, f_max (24.0 MHz), decision latency (728 clocks), window period measured on the board (2304 clocks at D = 0, 2240 (D+1) otherwise), peak rate 8,681/s, vectorless power 27 mW, energy per classification (3.1 µJ at D = 0) |

## Headline numbers

- Bit-exactness: CWRU 22,696 + MaFaulDa 526,246 + streaming 4,746 = **553,688 windows, 0 mismatches** against `ref_models.chain` (bins checked on 27,837 of them); every weight word read back equal; self-test bins/features/class identical to the RTL audit values.
- CWRU, 3 hp held out: 75.9 % (2.29 kS/s), 65.1 % (4.0), 80.5 % (8.0) window accuracy; sequence majority 90 / 80 / 100 %.
- MaFaulDa, 590 of 1,951 sequences held out: 71.7 % (2.29 kS/s), 63.9 % (8.0), 67.6 % (tracked) window accuracy; balanced 73.6 / 66.5 / 73.5 %; identical to the host model.
- Detection: alarm 4 windows after a fault onset at N = 3 (112 ms at 2.29 kS/s, 32 ms at 8 kS/s); CWRU healthy data gives zero false alarms from N = 2; MaFaulDa healthy data (72 % recall) 13/42 blocks at N = 3 falling to 1/42 at N = 6.
- FPGA: macro 10,718 LUT / 5,640 FF / 6 DSP (FFT 8,762 LUT); WNS +8.38 ns at 20 MHz (f_max 24.0 MHz on the xc7z010); 27 mW PL power estimate (vectorless; FFT 21, NN 4). No measured power: the board has no current sense.
- Timing on the board (global timer): window period 2304 clocks at D = 0 and exactly 2240 (D+1) clocks otherwise — the test plan's predictions; 8,681 classifications/s peak; decision latency 728 clocks = 36.4 us (RTL audit, reproduced here).

Streaming data: `stream_cwru_d249.json`, `stream_cwru_d70.json`, `stream_mafaulda_d249_imbalance.json`, `stream_mafaulda_d249_bearing.json`; per-window CSVs of the replays are not committed (size) and regenerate with `replay.py`.

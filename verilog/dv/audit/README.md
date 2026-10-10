# Audit harness for the corrected RTL (branch `fix/audit-defects`)

The pre-silicon audit of the fabricated SenseEdge netlist (CI2605) found five defects
behind a passing regression suite. This branch corrects them in RTL. No second tape-out
is planned: the fabricated part is measured as it is, and the corrected RTL is verified
here against integer reference models and demonstrated on an FPGA behind the same board.

## The five corrections

| # | Defect on the fabricated part | File | Correction |
|---|---|---|---|
| 1 | Weight decoder took the weight index from the byte offset of the address and accepted byte lane 0 only; from the word-aligned Caravel bus only weights 0, 4, ..., 84 (22 of 212) were reachable. No reset, no read-back. | `wb_interface.v`, `nn_engine.v`, `senseedge_top.v` | 53 word registers at `0x20..0xF0`; byte lane *k* of word *w* is weight 4*w*+*k*; a word write is forwarded as up to four byte writes and acknowledged after the last; weights reset to zero; the window reads back word-wide. |
| 2 | Last product of every neuron evaluated unsigned (it shared an expression with the unsigned bias concatenation). | `nn_engine.v` | Every product is a separate signed multiply, sign-extended to 24 bits. |
| 3 | Feature bytes (unsigned, 0..255) read as signed. | `nn_engine.v` | Inputs are unsigned bytes. |
| 4 | Feature extractor paired each magnitude with the previous cycle's address: bands one bin late, bin 1 unused, DC credited to the high band. | `feature_extract.v` | Each magnitude is accumulated under the address that produced it. Bands 1-4, 5-10, 11-20, 21-31 as designed. One cycle shorter. |
| 5 | Capture stored bits [11:0] of the 16-bit SPI shift register, one position below the MCP3201 code: gain of two and wrap beyond ±1024 codes. | `spi_adc_if.v` | Stores bits [12:1] and subtracts 2048: sample = code − 2048 over the full range. |

Also corrected: `firmware/senseedge_fw.c` and `senseedge_regs.h` (word-wide weight load with
read-back verification, absolute-address register access), `ml/train_senseedge_fixed.py`
(hardware model now matches the corrected arithmetic), and
`verilog/dv/unit_tests/tb_senseedge_top.v` (loads weights as words, checks read-back and
the expected class instead of "non-zero confidence").

Not included: clock gating. The fabricated part has none; the energy of a gated design is
quoted in the paper only as a post-route estimate, and gating remains future work.

## Reference models

`ref_models.py` is the specification. `capture`, `features` and `nn_infer` are integer
functions written from the corrected RTL, and the experiments below check the RTL against
them bit for bit. The same `nn_infer` is the training-time model for any weights that are
to run on this RTL.

## Experiments (`python3 run_audit.py`)

Needs Icarus Verilog 12 and NumPy. Results of 9 Oct 2026:

| ID | What | Result |
|---|---|---|
| A | `nn_engine.v` through its byte weight port: 5 random weight sets × 400 random vectors, plus 400 feature-like vectors | 2400/2400 vectors exact in class, confidence, 16 hidden registers and 4 logits |
| B | Every ADC code 0..4095 through the data-sheet SPI model | 4096/4096 give sample = code − 2048 |
| C | One tone per bin, full chain | 32/32 windows: RTL features == `features(RTL bins)`; peak bin == stimulus bin; each tone lands only in its designed band; DC-only window puts nothing in any band |
| D | Wishbone only, word-aligned, as the management core sees it | 53/53 weight words read back as written (all 212 weights reachable); a byte-lane-1 write changes only lane 1; a write above the window is ignored; 8/8 windows: FEATURE_DATA == `features(FFT_DATA)` and CLASS == `nn_infer(features)` |
| E | Latency | 728 clocks from full buffer to decision (fabricated part: 729); FFT 481, features 34, network 207 plus hand-off |
| F | `ml/train_corrected_exact.py`: weights trained on the integer grid of `nn_infer` (synthetic generator, 1600/400 split) | 100 % train, 99.2 % validation on the model; 400/400 validation vectors bit-exact in RTL; these are the weights in `ml/senseedge_weights.hex` and `firmware/nn_weights.h` |

The repository's seven unit testbenches pass on the corrected RTL (47 checks; the
integration test now asserts the read-back and the class).

## Synthesis check

`yosys -p "read_verilog ...; synth -top senseedge_top; stat"` on both RTL sets, same script:

| | Cells | Flip-flops | Latches |
|---|---|---|---|
| fabricated RTL (origin/main) | 43,997 | 7,033 | 0 |
| corrected RTL (this branch) | 46,375 | 7,440 | 0 |

The difference is the weight reset, the four-lane write sequencer, and the word-wide
read mux. No process-specific cell is used, so the RTL ports to an FPGA unchanged
(the twiddle ROM is an `initial` block; the 20 MHz Wishbone clock is the only clock).

## Pre-FPGA simulations beyond RTL (`python3 run_gl.py`, `cf verify senseedge_wb`)

| ID | What | Result (9 Oct 2026) |
|---|---|---|
| G1 | `nn_engine.v` synthesised by Yosys to generic gates, simulated with Yosys's cell models | 2400/2400 vectors: class and confidence exact |
| G2 | `senseedge_top` synthesised to generic gates, observed through the bus only | 53/53 words read back; 8/8 windows: features and class equal the reference; bins, features and classes identical to the RTL run |
| G3 | `senseedge_top` mapped to `sky130_fd_sc_hd` (dfflibmap + abc with the PDK liberty), simulated with the PDK's own cell models in functional mode | same: 53/53 words; 8/8 windows equal the reference and identical to the RTL run |
| S | seven stress scenarios of `tb_stress.v` (back-to-back master, weight rewrite during capture, reset mid-window, read ports hammered while busy, empty-select and out-of-window writes, divider 0) on RTL, generic netlist and sky130 netlist | 7/7 scenarios pass on all three: identical bins, features and class to the baseline, IRQ flag set and cleared, no stall; divider 0 completes a window in 3030 clocks |
| CV2 | Caravel co-simulation 2 (`cf verify senseedge_wb2`): through the management core, weight set A, four fault windows (class 1 each; the alarm asserts on the fourth with $N=3$ and the alarm pad goes high), then healthy windows (class 0, alarm cleared, pad low), then weight set B loaded over the bus and the same bin-8 record classified 2: a decision changed by a register-map update alone | **PASS**, 10 windows, 17.7 ms simulated |
| CV3 | Caravel co-simulation of the unmodified production firmware (`cf verify senseedge_fw`), its bit-banged UART decoded on GPIO 5 by the test side: banner, weight read-back report, and one CLASS line per window checked against the RTL prediction | **PASS**: banner, `WEIGHTS verified, word errors: 0`, then `CLASS:HEALTHY CONF:255 ALARM:0` for each of 3 windows, exactly as predicted on the RTL for that record; 61.5 ms simulated |
| CV | Caravel co-simulation (`cf verify senseedge_wb`): the real management core as Wishbone master running `senseedge_wb.c`, which loads 53 weight words, reads them back, runs one window on a data-sheet-timed ADC model driven from the cocotb side, and compares all 32 bins, 8 features and the class register with the RTL values | **PASS**: 53 words equal, bins/features/class (1, 255) equal, 357,471 cycles |

The co-simulation is the strongest pre-FPGA evidence: it exercises the only bus master the
silicon and the FPGA demo will have, with its word-aligned addresses and byte selects, and
the firmware helpers' offset semantics are bypassed by absolute-address access.

Three further defects of the shipped production firmware surfaced only when it was
compiled for and run on the real core (it never had been). The bit-banged UART called the
GPIO write helpers with two arguments where they take one data word, and left pad 5 in user
mode where the management core cannot drive it. The decimal printer used division, which
the rv32i build cannot link. And once those were fixed, the bit-banged bit time was found
to vary between 7.5k and 11.8k cycles from bit to bit (measured on the pad in co-simulation),
because the core executes from SPI flash without an instruction cache, so no receiver could
lock to it. The firmware now uses the management core's hardware UART on pad 6; the board
must wire the radio's RX to pad 6 (the test plan had already asked for this swap).

Lesson recorded while running these: the gate-level runner and the RTL runner must not
share a build directory. A stimulus file overwritten mid-run produced unknown values that
looked like a netlist bug and were not (`run_gl.py` now builds in `build/gl/` and both
benches refuse a short `codes.hex`).

## FPGA demonstration (to be run on hardware)

1. Instantiate `senseedge_top` behind a Wishbone master with word-aligned addresses and
   byte selects (a soft RISC-V or a bridge from a host). Drive `wb_clk_i` at 20 MHz.
2. Connect pads 0-2 to the node's ADC (MISO, SCLK, CS_N), pad 3 to the alarm LED.
3. Load `ml/senseedge_weights.hex` (212 bytes) as 53 words with `SE_NN_WEIGHT_WORD(w)`,
   read every word back.
4. Replay `build/codes.hex` records through an SPI emulator first and compare FFT_DATA,
   FEATURE_DATA and CLASS with `run_audit.py`'s expectations; then run on the motor.
5. Retrain for the machine with `ml/train_senseedge_fixed.py` (or the integer trainer)
   and reload over the bus: that is the register-map model update of the paper.

# SenseEdge on the Digilent Zybo Z7-10

The corrected SenseEdge RTL (branch `fix/audit-defects`) deployed on a Zybo Z7-10, with the
Zynq's ARM core playing the part of the Caravel management core. It is self-checking: an
on-chip MCP3201 emulator replays the audit's records, so the first run needs nothing but the
board and a micro-USB cable, and reports PASS when the bins, features and class match the
RTL values bit for bit. Flip SW0 and the same bitstream reads the real ADC on Pmod JE with
the shipped weights.

```
fpga/zybo_z7_10/
  rtl/axil_senseedge.v     AXI4-Lite slave -> Wishbone bridge, emulator, LEDs, around senseedge_top
  rtl/mcp3201_emu.v        MCP3201 emulator fed from a 4096-code RAM written over AXI
  constraints/zybo_z7_10.xdc
  scripts/build.tcl        Vivado batch build -> build/senseedge_zybo.xsa and .bit (+ utilization, timing, power reports)
  scripts/build_zybo.tcl   same, but first installs the Digilent board files from the AMD board store if missing
  scripts/vitis_build.py   scripted Vitis build of the program: vitis -s scripts/vitis_build.py [build/senseedge_zybo.xsa]
                           (SE_BAUD=921600 in the environment builds the fast-UART variant)
  scripts/program.tcl      xsdb: bitstream + ps7_init + ELF over JTAG
  sw/senseedge_zybo.c      bare-metal program for the PS (Vitis standalone)
  sim/tb_axil_senseedge.v  Icarus simulation of the wrapper driven exactly like the program
  sim/run_sim.sh
  host/run_board_tests.py  the whole board session of TEST_RUNS.md (runs 2-7) in one command
  host/run_mafaulda.py     MaFaulDa end to end: archives -> records -> weights -> board replay
  host/paper_figures.py    the paper's figures (PDF + PNG) from host/results/
  host/paper_tables.py     the paper's LaTeX tables from host/results/ and build/
```

Scripted flow on a machine with Vivado + Vitis (tested with 2026.1 on Windows, see TEST_RUNS.md):

```
cd fpga/zybo_z7_10
vivado -mode batch -source scripts/build_zybo.tcl              # ~6 min
vitis -s scripts/vitis_build.py                                 # build/senseedge_zybo.elf, build/ps7_init.tcl
xsdb scripts/program.tcl build/senseedge_zybo.bit build/ps7_init.tcl build/senseedge_zybo.elf
python host/run_board_tests.py --port COM10                     # results, figures and tables under host/results/
```

## What has been verified, and what has not

- `sim/run_sim.sh` (Icarus Verilog): the wrapper driven by an AXI-Lite master through the same
  sequence as `sw/senseedge_zybo.c`: id, record into the emulator, 53 weight words written and
  read back, a byte-strobe write, then bins, features and class on two windows. PASS, values
  identical to the RTL audit (`verilog/dv/audit`).
- Yosys `synth_xilinx` estimate for xc7z010: 9.6 k LUTs of 17.6 k, 6.1 k flip-flops, 3 RAMB18,
  7 DSP48E1. No latches.
- Not run here: Vivado and Vitis themselves (no Vivado on the authoring machine). The TCL
  follows the standard PS + AXI-Lite module-reference flow; if a command differs in your
  Vivado version, the equivalent GUI steps are listed below.

## Build the hardware

Requirements: Vivado 2022.1 or newer with Zynq-7000 support, and ideally the Digilent board
files (clone https://github.com/Digilent/vivado-boards and copy `new/board_files/*` into
`<Vivado>/data/boards/board_files/`). Without them the script configures the PS by hand.

```
cd fpga/zybo_z7_10
vivado -mode batch -source scripts/build.tcl
```

Outputs: `build/senseedge_zybo.xsa` (platform with bitstream), `build/senseedge_zybo.bit`,
`build/utilization.txt`, `build/timing.txt`. The chip runs at the PS fabric clock FCLK_CLK0 =
20 MHz, the same clock as the silicon; the AXI-Lite slave sits at 0x43C0_0000.

GUI equivalent, if needed: new project for xc7z010clg400-1 (board Zybo Z7-10); add the RTL
files listed in the script and the XDC; create a block design; add ZYNQ7 Processing System,
run block automation with the board preset; set PL fabric clock 0 to 20 MHz, enable M_AXI_GP0
and one fabric interrupt; add `axil_senseedge` as a module (Add Module); run connection
automation for `s_axi`; make `spi_miso`, `spi_sclk`, `spi_cs_n`, `led[3:0]`, `sw[1:0]`
external with exactly these names; connect `irq` to IRQ_F2P; set the slave's address to
0x43C00000, 64 K; validate; create the HDL wrapper; generate the bitstream; export the
hardware with the bitstream.

## Program and run

1. Vitis: *Create Platform Project* from `build/senseedge_zybo.xsa` (standalone, ps7_cortexa9_0),
   build it. *Create Application Project* on that platform (empty C), add
   `sw/senseedge_zybo.c`, build.
2. Set the board's boot jumper to JTAG, connect the micro-USB (PROG/UART), open a terminal on
   the USB-UART at 115200 8N1.
3. SW0 down. *Run As > Launch on Hardware*. Expected output:

```
SenseEdge on Zybo Z7-10, wrapper id 0x5e5e0002, protocol 1
SW0=0: on-chip emulator self-test
weights: 53 words written, 0 read-back errors
self-test: 32 bins, 8 features, class: PASS, identical to the RTL audit values
*** SELF-TEST PASSED ***
CLASS:BEARING_WEAR CONF:255 ALARM:0
CLASS:BEARING_WEAR CONF:255 ALARM:0
...
```

The record is a tone at bin 8 under the diagonal weight image (hidden k = band k, class k =
hidden k), so class 1 is the expected decision; LD0 (alarm) lights after four consecutive
fault decisions with N = 3, LD2 shows the emulator is selected, LD3 toggles per decision.

## Run on the sensor

Wire the node's ADC (or any MCP3201) to Pmod JE: JE1 = DOUT, JE2 = CLK, JE3 = CS, JE5 = GND,
JE6 = 3V3. Set SW0 up and reset the program (or power-cycle). The program then loads
`firmware/nn_weights.h`, sets the divider to 249 (2.29 kS/s at 20 MHz, the chip's reset value)
and prints one `CLASS:... CONF:... ALARM:...` line per window, exactly as the Caravel
firmware does. To change the model: retrain (`ml/train_corrected_exact.py` or
`ml/train_senseedge_fixed.py` on recorded features), run `ml/export_weights.py`, rebuild the
application. The register-map update of the paper is the same 53-word write at runtime.

## Register map of the wrapper (byte offsets from 0x43C0_0000)

| Offset | Register |
|---|---|
| 0x0000-0x00FF | the chip's Wishbone registers, unchanged (see `firmware/senseedge_regs.h`) |
| 0x1000 | EMU_CTRL: bit 0 = 1 selects the emulator, 0 the Pmod ADC; bit 1 = write 1 to reset the chip and the emulator (7 clocks, reads back 1 while active; weights, divider and alarm config must be reloaded afterwards) |
| 0x1004 | EMU_LEN: codes played before wrapping (1..4096) |
| 0x1008 | ID = 0x5E5E0002 |
| 0x100C | bits 1:0 = SW1:0; bits 15:8 = next emulator code index |
| 0x2000-0x5FFF | emulator code RAM, one 12-bit code per word |

To replay any record from the audit harness (`verilog/dv/audit/build/codes.hex`), write its
codes to 0x2000 onward, set EMU_LEN to the number of codes, and read bins and features back
through 0x10 and 0x14; the expected values come from `verilog/dv/audit/ref_models.py`.

## Dataset replay from a host PC (no sensor needed)

The program answers a binary protocol on the same UART (see the header of
`sw/senseedge_zybo.c` and `host/se_link.py`); `host/README.md` describes the pipeline that
turns a public vibration dataset (MaFaulDa, CWRU) into replay records, trains the weights on
the bit-exact host model, replays every test window through the board and checks it bit for
bit.  `sim/run_replay_check.py` runs the same replay in Icarus against the wrapper RTL, and
`host/test_protocol_mock.py` runs the protocol against a mock chip, so both ends are verified
before the board is touched.

See `TEST_RUNS.md` for the ordered list of runs, what each one records, and which paper slot it fills.

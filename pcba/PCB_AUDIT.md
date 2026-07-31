# SenseEdge PCBA — pre-assembly audit

Reviewed against the Caravel electrical contract and the design files in this
folder. **Verdict: the board as laid out in `SenseEdge_AI.kicad_pcb` will
work** — power, reset, boot flash, clock, and the analog sense chain are all
electrically correct. There is **one must-fix documentation hazard** (the BOM
and two markdown notes disagree with the layout on the clock and the
accelerometer part); assemble to the **layout**, not the stale BOM text.

## Will it work? — yes, with the reconciliations below

| Subsystem | Status | Notes |
|---|---|---|
| 3.3 V rail (AP2112K-3.3) | OK | feeds vdda/vddio/analog; 600 mA ≫ need |
| 1.8 V rail (AP2112K-1.8) | OK | feeds vccd core; separate LDO, good |
| Decoupling | OK | 100 nF per Caravel power pin + 10 µF bulk per rail |
| Reset (RESETB) | OK | 10 kΩ pull-up + 100 nF + button = valid POR |
| Boot flash (W25Q32) | OK | on flash_csb/clk/io0/io1 — Caravel boots from it |
| Clock | **MUST VERIFY** | layout = 20 MHz **active oscillator** (correct); BOM/notes say 25 MHz **crystal** (wrong — see below) |
| Sense chain | OK (verify part) | analog accel → anti-alias RC → MCP3201 ADC → SPI → mprj_io[0/1/2] |
| USB-C power | OK | 5.1 kΩ CC1/CC2 pulldowns = compliant power sink |
| EPAD | OK | thermal pad to ground plane with vias |

## MUST-FIX #1 — clock: active oscillator, not a crystal, at 20 MHz

Caravel's `clock` pin is a **CMOS clock input**; it has **no on-chip crystal
oscillator (Pierce) amplifier**. A bare 2-pin crystal + load caps on that pin
**will not oscillate and the chip will not run.** It must be driven by an
**active oscillator**.

- The **PCB layout is correct**: it places `ECS-2520S33-200-FN-TR`, a 20 MHz
  CMOS oscillator. Assemble this part.
- The **BOM (`senseedge_bom.csv`) is wrong**: it lists `Y1 = ABM3B-25.000MHZ`
  (a 25 MHz passive crystal) with 22 pF load caps `C11/C12`. Populating that
  bricks the clock. It is also **25 MHz > the 20 MHz static-timing sign-off**,
  so even an active 25 MHz part would risk setup violations.
- `schematic_connections.md` is internally contradictory (pin 2 text says the
  20 MHz ECS oscillator; a later note says "Y1 (25 MHz) crystal + 22 pF").

**Action:** confirm the assembly BOM populates the **ECS-2520S33-200 (20 MHz
oscillator)**; delete `C11/C12` 22 pF load-cap entries if the footprint is the
3-pad oscillator; drive `clock` at **≤ 20 MHz**.

## MUST-FIX #2 — accelerometer part number is inconsistent

The docs name three different accelerometers: `ADXL1002` (±50 g, BOM),
`ADXL326` (±16 g, schematic_connections U2), `ADXL1002` again elsewhere. All
are analog-output MEMS and electrically drop into the same ADC front end, but
the **sensitivity (mV/g) and full-scale differ**, which changes the feature
scaling the NN was trained on. **Action:** pick one part, confirm its
sensitivity, and match the feature-extraction assumptions / retraining to it.

## MUST-FIX #3 — reference-designator drift between documents

`senseedge_bom.csv` and `schematic_connections.md` assign U1/U2/U3/J1 to
different components (e.g. Caravel is `U1` in the BOM but `U3` in the
connections doc; the ADC is `U2` in the BOM but `J1` in the connections doc).
This will confuse an assembly house. **Action:** regenerate the BOM and the
connections table from the single authoritative `SenseEdge_AI.kicad_pcb` so
all three agree, before releasing to assembly.

## Lower-priority checks

- Anti-alias RC (100 Ω + 100 nF ≈ 16 kHz corner) is fine for a ≤ few-kHz
  vibration band at ≤ 100 kSPS; confirm it matches your target band.
- Confirm the ADC `VREF = 3.3 V` matches the accelerometer's output swing so
  the full code range is used.
- Confirm `mprj_io` mapping on the board matches the RTL pin plan
  (`io[0]`=MISO in, `io[1]`=SPI_CLK out, `io[2]`=CS out, `io[3]`=ALARM,
  `io[4]`=STATUS) — the RTL drives exactly these.

See `TESTPROTOCOL.md` for the ordered bring-up and functional test sequence.

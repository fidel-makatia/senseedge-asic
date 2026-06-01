# SenseEdge: Predictive-Maintenance ASIC on SKY130

**Fidel Makatia** — PhD Student, Analog & Mixed-Signal Group (Advisor: Dr. Silva-Martinez)
Department of Electrical & Computer Engineering, Texas A&M University

---

## Executive Summary

SenseEdge is a custom mixed-signal Application-Specific Integrated Circuit (ASIC) designed for end-to-end vibration-based predictive maintenance in rotating machinery. The chip ingests a 3-axis MEMS accelerometer stream and outputs a fault-class decision — Healthy, Bearing Wear, Imbalance, or Misalignment — entirely from a dedicated hardware signal-processing chain (ADC interface → FFT → feature extractor → INT8 neural network → alarm logic), without invoking any instruction-fetch loop or external DSP/network resource. The design is hosted in the user-project area of the open-source Caravel SoC harness on the SKY130A process; the Caravel harness contributes a PicoRV32-based management RISC-V used only for chip boot, GPIO routing, and Wishbone-bus mastering, and is not in the inference path. The chip is currently undergoing a re-spin for the ChipFoundry CI2605 shuttle.

## Architectural Design

The signal chain is comprised of five custom datapath blocks, all operating at 20 MHz from a single CMOS clock source:

| Block Component | Primary Function | Technical Specifications |
|---|---|---|
| **SPI-ADC Interface** | 12-bit serial readout | Interfaces with external MCP3201; 3-axis multiplexed; Wishbone-mapped |
| **Hardware FFT Engine** | 64-point, radix-2 | Fixed-point processing; streaming butterfly architecture; single-cycle complex MAC |
| **Feature Extractor** | Spectral analysis | Generates 8 features (band energies, kurtosis, crest factor) using dedicated arithmetic logic (no microcode) |
| **NN Inference Engine** | Fault classification | 8 → 16 → 4 fully-connected; INT8 quantized; 212 B weights; runtime-updatable via Wishbone |
| **Alarm Logic & Interface** | Status indication | Threshold and persistence evaluation; LED drive and host readback; GPIO[5] alarm output; GPIO[6] reset |

**Classifiable Fault States:** Healthy, Bearing Wear, Imbalance, and Misalignment.

### Signal-Flow Pipeline

The full inference data path — from raw analog vibration to fault decision — executes entirely in dedicated hardware on a single 20 MHz clock domain, with no instruction-fetch loop in the critical path. The diagram below traces one inference cycle (~35 µs at 20 MHz):

```
   MEMS accelerometer (3-axis, analog)
            │
            ▼
   MCP3201 ADC (external) ── 12-bit serial ──▶ spi_adc_if.v
                                                  │  (samples → buffer)
                                                  ▼  64 samples / axis
                                            ┌─────────────────────────┐
                                            │      fft_engine.v       │
                                            │  64-pt radix-2 DIT      │
                                            │  Q1.14 twiddle ROM      │
                                            │  16-bit in / 24-bit     │
                                            │  internal / 16-bit out  │
                                            └─────────────────────────┘
                                                  │  32 magnitude bins
                                                  ▼
                                       ┌──────────────────────────────┐
                                       │     feature_extract.v        │
                                       │  4 band energies + peak freq │
                                       │  + peak magnitude + spectral │
                                       │  centroid + total energy     │
                                       └──────────────────────────────┘
                                                  │  8 × uint8 features
                                                  ▼
                                       ┌──────────────────────────────┐
                                       │        nn_engine.v           │
                                       │  Single MAC, time-multiplexed│
                                       │   L1: 16 neurons × 8 MACs    │
                                       │       + bias + ReLU          │
                                       │   L2:  4 neurons × 16 MACs   │
                                       │       + bias + argmax        │
                                       │  Weights (212 B) loaded once │
                                       │  at boot over Wishbone       │
                                       └──────────────────────────────┘
                                                  │  class_id (2-bit)
                                                  │  confidence (8-bit)
                                                  ▼
                                       ┌──────────────────────────────┐
                                       │       alarm_logic.v          │
                                       │  Threshold + N-sample        │
                                       │  persistence; LED drive +    │
                                       │  Wishbone status readback    │
                                       └──────────────────────────────┘
                                                  │
                                                  ▼
                                       GPIO[5] alarm LED + host readback
```

**Retasking via runtime-writable weights.** Because the 212 INT8 weights live in a Wishbone-writable register file inside `nn_engine.v` — not in ROM — the same silicon can be retargeted to a different rotating-machinery diagnostic (e.g., HVAC blower imbalance, pump cavitation, gearbox wear) simply by uploading a freshly-trained 212-byte weight blob at boot, with no hardware change. The eight hardware features (band energies + peak + centroid + total energy) are general enough to discriminate most vibration-domain fault signatures; the network simply re-learns the decision boundaries.

## Physical Layout

![SenseEdge user_project_wrapper layout](gds_new.png)

*GDSII view of the user_project_wrapper macro: senseedge_top is placed in the lower-center of the Caravel user area, surrounded by the wrapper's power distribution network. The 38 GPIO pins are labeled along the left and right edges.*

## Technical Implementation Metrics

| Metric | Specification |
|---|---|
| Process Technology / PDK | SKY130A (SkyWater 130 nm) |
| Die Area (User Macro) | ≈ 7.30 mm² (2920 × 2500 µm) |
| Standard Cell Library | `sky130_fd_sc_hd` (high-density) |
| **Total Placed Cells** | **1,033,187 instances, 7.21 mm² active area** |
| Active Logic Cells | 50,581 (sequential, combinational, buffers, repair) |
| &nbsp;&nbsp;&nbsp;&nbsp;Sequential (flip-flops) | 7,037 |
| &nbsp;&nbsp;&nbsp;&nbsp;Multi-input combinational | 34,561 |
| &nbsp;&nbsp;&nbsp;&nbsp;Clock-tree buffers / inverters | 1,785 |
| &nbsp;&nbsp;&nbsp;&nbsp;Timing-repair buffers | 6,779 |
| &nbsp;&nbsp;&nbsp;&nbsp;Datapath inverters / buffers | 419 |
| Tap / Endcap Cells | 102,712 |
| Antenna-Protection Cells | 54,761 |
| Decoupling-Capacitor & Fill Cells | 825,133 |
| Clock Domain | `wb_clk_i` @ 20 MHz (50 ns), single domain architecture |
| EDA Toolchain | LibreLane 2.4.6 → OpenROAD → Magic / KLayout / Netgen |
| Signoff Status | DRC clean (Magic + KLayout), LVS clean (unique circuit match), Antenna clean |
| Timing Analysis | +6.66 ns setup slack at typical corner (1.80 V, 25 °C) |
| Power Consumption | ~150 µW estimated active power (post-PnR IR-drop analysis) |
| Design Verification (DV) | 46 SystemVerilog assertions across 7 unit testbenches (0 failures); Gate-level simulations passed for all 4 scenarios |

## Machine Learning Pipeline

The on-chip neural network is the product of a complete, reproducible training pipeline that mirrors the hardware exactly. The pipeline is intentionally lightweight — a pure NumPy implementation of forward/backward propagation — so that every numerical operation can be traced through to the Verilog inference engine without dependency on a heavy framework.

**Dataset.** Training data is sourced from the **Case Western Reserve University (CWRU) bearing dataset**, the de-facto benchmark for rotating-machinery fault classification. Raw vibration time-series segments are passed through the same eight-feature extractor implemented in `feature_extract.v` — four spectral band energies (FFT bins 1–4, 5–10, 11–20, 21–31), peak frequency, peak magnitude, spectral centroid, and total energy — so the training set is generated from the exact features the hardware will see. A synthetic-data fallback is included for environments without the CWRU `.mat` files.

**Model.** A two-layer fully-connected network: 8 inputs → 16 ReLU-activated hidden units → 4 logit outputs (argmax selects the fault class). Total parameter count is **212** (W₁: 128 + b₁: 16 + W₂: 64 + b₃: 4) — small enough to fit comfortably in the chip's NN engine register file with no SRAM required.

**Training.** Mini-batch SGD with cross-entropy loss; default 200 epochs at learning rate 0.01 with batch size 64. Training tracks per-class validation accuracy and saves the best-performing weight set.

**Quantization.** After training, weights are converted to signed INT8 (range −128 to +127) using **symmetric min-max quantization** with a per-tensor scale factor. The pipeline then performs an INT8-vs-FP32 accuracy comparison on the held-out validation set; the design target is to keep the INT8 accuracy drop below 1 percentage point relative to the float baseline.

**Export.** Quantized weights are emitted both as a NumPy `.npz` archive (for software regression) and as a packed C header (`nn_weights.h`) that the on-chip firmware loads into the inference engine over Wishbone. The full pipeline is reproducible via `python train_senseedge.py --cwru-dir <path>` and is checked in alongside the RTL so the chip's behavior remains fully auditable.

## Boot and Runtime Operation

The full lifecycle from chip power-up to live fault classification involves three distinct phases, only one of which (boot) requires any participation from the Caravel management RISC-V.

### Phase 1 — Boot-Time Weight Upload (one-time, ≈ 1 ms)

At power-on, the on-board ESP32-C3 microcontroller releases the chip's reset, then streams the 212-byte INT8 weight array (compiled into the firmware as `nn_weights.h`) onto the chip. The Caravel management RISC-V acts as Wishbone master and fans these writes out to the user-project area, where they land in `nn_engine.v`'s weight register file. Once this transfer completes, the RISC-V plays no further role in the inference path:

```
   nn_weights.h  ──▶  ESP32-C3 firmware  ──UART/SPI──▶  Caravel mgmt RISC-V
                                                                │
                                                                ▼  Wishbone bus
                                                       212 single-byte writes
                                                       wt_wr_addr 0 … 211
                                                       wt_wr_data INT8
                                                                │
                                                                ▼
                                                  nn_engine.v weight register file
                                                    [  0..127] Layer 1 weights
                                                    [128..143] Layer 1 biases
                                                    [144..207] Layer 2 weights
                                                    [208..211] Layer 2 biases
```

This phase also enables **field retasking**: writing a different 212-byte blob (e.g., HVAC-blower-trained weights instead of bearing-fault weights) repurposes the same silicon for a different rotating-machinery diagnostic with no hardware change.

### Phase 2 — Runtime Inference (continuous, ≈ 35 µs per cycle)

After boot, the chip enters an autonomous inference loop driven entirely by the dedicated hardware datapath documented in the *Signal-Flow Pipeline* diagram above. The MCP3201 ADC delivers 12-bit samples over SPI; `spi_adc_if.v` collects 64 samples per axis into an on-chip buffer; `fft_engine.v` computes a 64-point radix-2 FFT and presents 32 magnitude bins; `feature_extract.v` derives the eight features the network was trained on; `nn_engine.v` performs the two MAC-and-activation passes (8 → 16 → 4) using its time-multiplexed single-MAC unit; and `alarm_logic.v` applies a threshold + N-sample persistence filter before driving the on-board LED and updating the Wishbone-readable status register.

| Sub-stage | Approx. cycles @ 20 MHz |
|---|---|
| ADC sample collection (64 samples) | 64 × *sample-period* |
| 64-point FFT (six butterfly stages, time-multiplexed) | ≈ 400 |
| 8-feature extraction | ≈ 80 |
| NN inference (212 single-MAC operations + ReLU + argmax) | ≈ 250 |
| Alarm logic update | ≈ 10 |
| **End-to-end** | **≈ 35 µs** (excluding ADC sample window) |

Because no instruction fetch or memory hierarchy lies on the critical path, throughput is bounded only by the chosen sample rate, not by computation. The signal chain can sustain inference rates well into the kHz range — orders of magnitude faster than the mechanical bandwidths of interest in rotating-machinery diagnostics.

### Phase 3 — Host Readback and Control (on-demand)

The Caravel management RISC-V remains available throughout runtime to service Wishbone reads from a host. The user-project register map exposes:

- The latest `class_id` (2-bit fault class) and `confidence` (8-bit max activation)
- Per-class output activations (useful for confidence diagnostics and logging)
- The alarm state and persistence counter
- Re-write access to the NN weight register file (for field updates without re-fabrication)

This division of labor — RISC-V for boot, configuration, and host I/O; dedicated hardware for the µW-class inference loop — gives SenseEdge both the programmability advantages of a managed SoC and the energy-efficiency advantages of an all-hardware ASIC datapath.

## System Ecosystem & Deliverables

A comprehensive demonstration platform has been developed to accompany the ASIC, including:

- **Hardware:** A custom KiCad PCB featuring USB-C power delivery, an ESP32-C3 microcontroller, onboard flash memory, and dedicated 1.8 V/3.3 V LDO regulators.
- **Mechanical:** A FreeCAD-designed enclosure (available in STEP/STL formats).
- **Software:** C firmware providing an on-chip register-map interface and weight-loading flow over Wishbone.
- **Documentation:** A complete training pipeline and an 8-slide system overview presentation.

## ChipFoundry Reference Application Design Contest

SenseEdge is a participating and winning entry in the ChipFoundry Reference Application Design Contest. This competition challenges engineers to build complete, fully open-source hardware solutions — including custom silicon (utilizing the Caravel SoC harness), PCBA, firmware, and mechanicals — for Industrial, Commercial, or Edge-IoT sectors.

Evaluated by an expert judging panel (featuring industry veterans **David Tupman**, **Josh Lifton**, and **Mohamed Kassem**), SenseEdge was assessed on technical innovation, verification rigor, documentation quality, and overall feasibility. After securing a perfect 10/10 rating at the proposal stage, SenseEdge was selected as a **Primary Winner**.

The contest prize fully funds the project's transition to physical hardware, awarding:

- **100% Sponsored Fabrication:** A fully funded slot on the May chipIgnite shuttle.
- **Silicon Allocation:** 50 QFN-packaged SoC parts returned to the designer.
- **Prototyping Support:** Financial and logistical assistance for initial PCBA manufacturing and enclosure fabrication (3D printing / CNC).
- **Global Showcase:** Feature placement as an official ChipFoundry reference architecture with full designer attribution.

The total prize package is valued at **approximately $10,000–$15,000 USD** at commercial open-PDK MPW pricing — covering mask preparation, fabrication, dicing, packaging, return shipping of bonded samples, and prototyping support. The award effectively eliminates the single largest line item in a graduate-level tape-out budget and moves the project from concept to characterized silicon at zero out-of-pocket cost to the student.

## Value Proposition & Market Significance

| Metric | Industry Standard Solutions | SenseEdge Innovation |
|---|---|---|
| Hardware Cost | $500 – $5,000 per analyzer node | ~$13 Bill of Materials (BOM) at 100-unit volume |
| Operational Cost | ~$50 / month / asset (Cloud-ML services) | Localized processing; zero subscription fees |
| System Overhead | 50–100 mW power draw; requires firmware development | ~150 µW power draw; plug-and-play functionality |

The significant reduction in both cost and power consumption enables the deployment of predictive maintenance in previously cost-prohibitive sectors, including small-scale manufacturing facilities, municipal water / wastewater utilities, commercial HVAC fleets, and critical infrastructure in resource-constrained environments.

## Project Status & Milestones

| Milestone Event | Date | Outcome |
|---|---|---|
| Proposal Evaluation | March 25, 2026 | Shortlisted (Rating: 10/10 from M. Kassem, ChipFoundry) |
| RTL Freeze & DV Signoff | April 30, 2026 | Signoff complete (Clean) |
| Official Shuttle Tape-out Deadline | May 13, 2026 | Accepted; full fabrication sponsorship awarded |
| ChipFoundry Application Design Contest | May 2026 | **Primary Winner (Fab Sponsored)** |
| Server-side Physical Verification (PV) | May 23, 2026 | Re-spin required (decoupling-cell density specification) |
| Re-spin Submission | May 27, 2026 | Uploaded; precheck clean; awaiting final PV |

**Next Steps:** Silicon return, system assembly, and board-level characterization are anticipated in October / November 2026.

---

*For further technical details and access to the design repository, please visit: [github.com/fidel-makatia/senseedge-asic](https://github.com/fidel-makatia/senseedge-asic)*

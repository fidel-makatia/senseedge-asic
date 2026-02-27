<div align="center">

<img src="https://umsousercontent.com/lib_lnlnuhLgkYnZdkSC/hj0vk05j0kemus1i.png" alt="ChipFoundry Logo" height="140" />

[![Typing SVG](https://readme-typing-svg.demolab.com?font=Inter&size=44&duration=3000&pause=600&color=4C6EF5&center=true&vCenter=true&width=1100&lines=SenseEdge;Predictive+Maintenance+ASIC;Hardware+FFT+%2B+Neural+Network;Open-Source+%7C+SKY130)](https://git.io/typing-svg)

[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)
[![ChipFoundry Marketplace](https://img.shields.io/badge/ChipFoundry-Marketplace-6E40C9.svg)](https://platform.chipfoundry.io/marketplace)

</div>

## Table of Contents
- [Overview](#overview)
- [Project Description](#project-description)
- [System Architecture](#system-architecture)
- [Silicon Design](#silicon-design)
- [PCBA Design](#pcba-design)
- [Firmware](#firmware)
- [Mechanical Enclosure](#mechanical-enclosure)
- [Bill of Materials](#bill-of-materials)
- [Project Timeline](#project-timeline)
- [Deliverables](#deliverables)
- [Documentation & Resources](#documentation--resources)
- [Prerequisites](#prerequisites)
- [Project Structure](#project-structure)
- [Starting Your Project](#starting-your-project)
- [Development Flow](#development-flow)
- [GPIO Configuration](#gpio-configuration)
- [Local Precheck](#local-precheck)
- [Checklist for Shuttle Submission](#checklist-for-shuttle-submission)

## Overview

**SenseEdge** is a complete reference design — custom ASIC, PCB, firmware, and enclosure — for real-time vibration-based predictive maintenance at the edge. The custom silicon integrates a **64-point radix-2 FFT engine**, a **tiny neural network inference engine** (8→16→4, INT8), and a **feature extraction pipeline** into the Caravel user project area.

The system classifies machine health into four states — **Healthy, Bearing Wear, Imbalance, and Misalignment** — entirely in hardware, with no cloud connectivity required. At volume, the complete sensor node costs under **$15** — a fraction of the $500–$5,000 charged by commercial solutions.

This repository contains SenseEdge's user project designed for integration into the **Caravel chip user space**, utilizing:

* **IO Pads:** SPI interface to external ADC, alarm GPIO output, status LED.
* **Logic Analyzer Probes:** 128 signals for classification result monitoring and debug.
* **Wishbone Port:** Register-mapped control interface for configuration, weight loading, FFT/feature readback, and interrupt handling.

---

## Project Description

### Problem

Unplanned equipment downtime costs industrial facilities an estimated **$50 billion annually**. Traditional vibration monitoring systems rely on expensive proprietary hardware ($500–$5,000 per sensor node) or cloud-connected solutions with latency and privacy concerns.

Existing approaches fall short in three ways:

1. **Software-based FFT on microcontrollers** is slow and power-hungry, limiting sampling rates and battery life
2. **Cloud-dependent ML inference** introduces latency, connectivity requirements, and recurring subscription costs
3. **Fixed-threshold alerting** generates excessive false alarms and requires manual calibration for every machine

There is no low-cost, open-source solution that performs both spectral analysis and intelligent fault classification entirely in hardware at the sensor node.

### Solution

SenseEdge moves the entire vibration analysis pipeline into dedicated silicon:

- A **64-point radix-2 FFT engine** for hardware-accelerated spectral analysis
- A **feature extraction pipeline** that reduces 32 FFT bins to 8 meaningful spectral features
- A **tiny neural network** (8→16→4, INT8 fixed-point) for on-chip fault classification
- **Configurable alarm logic** with consecutive-fault filtering to reduce false positives

The hardware pipeline runs autonomously — ADC samples arrive via SPI, pass through FFT, get reduced to features, and are classified by the neural network — all without CPU intervention. The Caravel RISC-V core handles only configuration, weight loading, and result reporting.

### Target Applications

| Sector | Use Case |
|---|---|
| Manufacturing | Motor, pump, and compressor monitoring on factory floors |
| HVAC | Fan and compressor health in commercial buildings |
| Energy | Wind turbine gearbox and generator monitoring |
| Water/Wastewater | Pump station condition monitoring |
| Agriculture | Irrigation pump and grain dryer motor health |

---

## System Architecture

```
┌──────────────────────────────────────────────────────────────┐
│                       CARAVEL SoC                            │
│                                                              │
│  ┌──────────┐         Wishbone Bus                           │
│  │ RISC-V   │◄════════════════════════════════════╗          │
│  │ CPU      │  (config, weight loading,           ║          │
│  └──────────┘   result readback)                  ║          │
│                                                   ║          │
│  ┌────────────────────────────────────────────────────────┐  │
│  │              USER PROJECT AREA (SenseEdge)              │  │
│  │                                                         │  │
│  │  ┌───────────┐    ┌───────────┐    ┌───────────────┐   │  │
│  │  │ SPI ADC   │───▶│ 64-Point  │───▶│ Feature       │   │  │
│  │  │ Interface │    │ Radix-2   │    │ Extraction    │   │  │
│  │  │           │    │ FFT       │    │ (8 features)  │   │  │
│  │  └───────────┘    └───────────┘    └───────┬───────┘   │  │
│  │                                            │           │  │
│  │  ┌───────────────┐    ┌────────────────────▼────────┐  │  │
│  │  │ Wishbone B4   │◄───│ Neural Network              │  │  │
│  │  │ Slave + Regs  │    │ Inference Engine             │  │  │
│  │  │               │    │ (8→16→4 FC, INT8, ReLU)     │  │  │
│  │  └───────────────┘    └────────────────────┬─────────┘ │  │
│  │                       ┌────────────────────▼─────────┐ │  │
│  │                       │ Alarm Logic + IRQ            │──▶ GPIO
│  │                       └──────────────────────────────┘ │  │
│  └────────────────────────────────────────────────────────┘  │
└──────────────────────────────────────────────────────────────┘
```

---

## Silicon Design

### RTL Modules

#### 1. SPI ADC Interface — `spi_adc_if.v`
- SPI master for MCP3201-style 12-bit ADC
- Configurable sample rate via programmable clock divider (up to 100 kSPS)
- 64-sample circular buffer with automatic buffer-full signaling
- 12-bit ADC data sign-extended to 16-bit for FFT input

#### 2. 64-Point Radix-2 FFT Engine — `fft_engine.v`
- Decimation-in-time with bit-reversal input addressing
- Fixed-point: 16-bit input, 24-bit internal precision, 16-bit magnitude output
- Pre-computed twiddle factor ROM (Q1.14 format, 32 entries)
- Single butterfly unit, time-multiplexed across 6 stages x 32 butterflies
- Fast magnitude approximation: `max(|Re|,|Im|) + 0.5*min(|Re|,|Im|)`
- Outputs 32 magnitude bins (DC to Nyquist)

#### 3. Feature Extraction Engine — `feature_extract.v`
Computes 8 spectral features from 32 FFT bins:

| Feature | Description |
|---|---|
| Band Energy (x4) | Sum of bins in low, mid-low, mid-high, and high frequency bands |
| Peak Frequency | Bin index with maximum magnitude |
| Peak Magnitude | Value at peak bin |
| Spectral Centroid | Weighted average frequency |
| Total Energy | Sum across all bins |

All features normalized to 8-bit unsigned for NN input.

#### 4. Neural Network Inference Engine — `nn_engine.v`
- Fully-connected: **8 inputs → 16 hidden (ReLU) → 4 outputs (argmax)**
- INT8 weights and activations
- Single MAC unit, time-multiplexed: 192 MAC operations per inference
- Weights loadable at runtime via Wishbone (field-updateable models)
- Total parameters: **(8x16) + 16 + (16x4) + 4 = 212 bytes**
- Output: 2-bit class ID + 8-bit confidence score

#### 5. Wishbone Slave Interface — `wb_interface.v`
- 32-bit Wishbone B4 compliant slave
- Register map:

| Offset | Register | Access | Description |
|---|---|---|---|
| 0x00 | CTRL | R/W | Enable, mode, sample rate divider |
| 0x04 | STATUS | R | FSM state, busy flags, alarm status |
| 0x08 | CLASS_RESULT | R | 2-bit class ID + 8-bit confidence |
| 0x0C | ALARM_CFG | R/W | Threshold, consecutive fault count |
| 0x10 | FFT_DATA | R | Auto-incrementing FFT bin readback |
| 0x14 | FEATURE_DATA | R | Auto-incrementing feature readback |
| 0x18 | IRQ_FLAGS | R/W | Interrupt status and clear |
| 0x1C | CLK_DIV | R/W | ADC sample rate divider |
| 0x20-0x74 | NN_WEIGHTS | W | Neural network weight registers |

#### 6. Alarm & Interrupt Logic — `alarm_logic.v`
- Configurable confidence threshold for fault detection
- Consecutive fault counter (N faults before alarm — reduces false positives)
- GPIO output for direct hardware alarm (LED, buzzer)
- Single-cycle IRQ pulse to RISC-V for firmware handling

### Area Estimate

| Block | Est. Gates | Est. Area (mm^2) |
|---|---|---|
| SPI ADC Interface | 3,000 | 0.4 |
| FFT Engine (64-pt) | 18,000 | 2.5 |
| Feature Extraction | 4,000 | 0.5 |
| NN Inference Engine | 10,000 | 1.5 |
| Wishbone Interface + Regs | 5,000 | 0.7 |
| Alarm & IRQ Logic | 2,000 | 0.3 |
| **Total** | **~42,000** | **~5.9** |
| **Caravel User Area** | | **10.0** |
| **Margin** | | **~4.1 (41%)** |

### Target Specifications

| Parameter | Value |
|---|---|
| Process | SkyWater SKY130 (130nm) |
| Target Clock | 25 MHz |
| FFT Size | 64-point radix-2 DIT |
| ADC Sample Rate | Up to 100 kSPS |
| Frequency Resolution | ~1.5 kHz at 100 kSPS |
| NN Precision | INT8 weights and activations |
| NN Parameters | 212 (runtime-loadable) |
| Classification Classes | 4 (Healthy, Bearing Wear, Imbalance, Misalignment) |
| Inference Latency | < 10 us (192 MACs at 25 MHz) |
| Power (estimated) | < 5 mW (digital logic at 1.8V) |

### Verification Plan

| Level | Tool | Scope |
|---|---|---|
| Unit testbenches | Icarus Verilog | Each RTL module individually |
| FFT accuracy | Icarus + Python | Compare hardware FFT output against NumPy FFT |
| NN inference accuracy | Icarus + Python | Verify hardware classification matches Python INT8 inference |
| Full-chip integration | Cocotb/Verilator | End-to-end: SPI stimulus → FFT → features → NN → alarm |
| Gate-level simulation | Icarus Verilog | Post-synthesis netlist with SDF timing |
| STA | OpenSTA | Timing closure at 25 MHz |
| DRC/LVS | Magic VLSI | Physical verification |
| Precheck | cf precheck | Tapeout readiness |

---

## PCBA Design

A compact sensor node PCB designed in **KiCad**, intended to bolt directly onto industrial equipment.

| Component | Part | Purpose |
|---|---|---|
| ASIC | SenseEdge (Caravel QFN-64) | FFT + NN inference engine |
| Accelerometer | ADXL345 (SPI) | 3-axis vibration sensing |
| Temperature | TMP117 (I2C) | Ambient/surface temperature |
| Wireless | ESP32-C3-MINI (UART) | BLE/Wi-Fi for remote dashboard |
| Power | TPS62823 | 3.3V / 1.8V dual-rail regulation |
| Power Input | USB-C or 12-24V industrial | Flexible deployment power |
| Connector | M12-A 4-pin | Industrial sensor cable standard |

**Board specs:** 45mm x 35mm, 2-layer FR4, M3 mounting holes for equipment attachment or DIN-rail clip.

---

## Firmware

The Caravel RISC-V core runs lightweight C firmware:

1. **Boot & Initialization** — Configure ADC sample rate, load 212 pre-trained INT8 weights into NN registers, set alarm thresholds
2. **Runtime** — Hardware pipeline runs autonomously; CPU handles IRQ on classification events, reads results, transmits via UART to ESP32
3. **Weight Update** — Receive new model weights over UART for field-updateable intelligence without silicon changes

### ML Training Pipeline (Offline)

```
CWRU Bearing Dataset → 64-pt FFT → Feature Extraction → Train FC (8→16→4)
                                                          with QAT (INT8)
                                                              ↓
                                                   Export 212 bytes → Load via Firmware
```

- **Dataset**: [CWRU Bearing Data Center](https://engineering.case.edu/bearingdatacenter) — industry standard benchmark
- **Framework**: TensorFlow Lite with INT8 quantization-aware training
- **Expected accuracy**: >90% on 4-class classification

---

## Mechanical Enclosure

- Designed in **FreeCAD**, IP54-rated snap-fit enclosure
- 55mm x 45mm x 25mm, 3D printed ASA/PETG (industrial temperature range)
- Cable gland for M12 connector, mounting ears with M3 bolts, LED light pipe

---

## Bill of Materials

| Item | Cost (qty 1) | Cost (qty 100) |
|---|---|---|
| SenseEdge ASIC (QFN) | Sponsored | Sponsored |
| ADXL345 Accelerometer | $3.50 | $2.10 |
| ESP32-C3-MINI | $2.80 | $1.90 |
| TMP117 Temp Sensor | $2.20 | $1.60 |
| TPS62823 Regulator | $1.10 | $0.75 |
| Connectors (USB-C + M12) | $2.90 | $2.05 |
| PCB (2-layer) | $5.00 | $0.80 |
| Passives, LED, misc | $2.00 | $1.30 |
| 3D Printed Enclosure | $4.00 | $2.50 |
| **Total** | **~$23.50** | **~$13.00** |

This is **20-400x cheaper** than commercial vibration monitoring solutions ($500–$5,000+).

---

## Project Timeline

| Week | Dates | Milestone |
|---|---|---|
| 1-2 | Feb 26 - Mar 11 | RTL design: all 7 modules |
| 3 | Mar 12 - Mar 18 | Unit + integration testbenches |
| 4 | Mar 19 - Mar 25 | **Proposal submission** |
| 5-6 | Mar 26 - Apr 8 | OpenLane synthesis, place & route, STA |
| 7 | Apr 9 - Apr 15 | Gate-level simulation, precheck |
| 8 | Apr 16 - Apr 22 | KiCad PCBA layout, firmware |
| 9 | Apr 23 - Apr 29 | Mechanical enclosure, demo video |
| 10 | Apr 30 | **Final submission** |

---

## Deliverables

- [ ] Complete RTL source (Verilog) with unit and integration testbenches
- [ ] Verified GDSII layout passing OpenLane chipIgnite flow and precheck
- [ ] Gate-level simulation results with SDF back-annotation
- [ ] STA timing reports at 25 MHz
- [ ] KiCad PCBA design (schematic, layout, BOM, Gerbers)
- [ ] FreeCAD mechanical enclosure (STEP, STL for printing)
- [ ] RISC-V firmware source (C)
- [ ] Python training pipeline with CWRU dataset
- [ ] 3-minute demonstration video
- [ ] Full documentation enabling third-party replication

---

## Documentation & Resources
For detailed hardware specifications and register maps, refer to the following official documents:

* **[Caravel Datasheet](https://github.com/chipfoundry/caravel/blob/main/docs/caravel_datasheet_2.pdf)**: Detailed electrical and physical specifications of the Caravel harness.
* **[Caravel Technical Reference Manual (TRM)](https://github.com/chipfoundry/caravel/blob/main/docs/caravel_datasheet_2_register_TRM_r2.pdf)**: Complete register maps and programming guides for the management SoC.
* **[ChipFoundry Marketplace](https://platform.chipfoundry.io/marketplace)**: Access additional IP blocks, EDA tools, and shuttle services.

---

## Prerequisites
Ensure your environment meets the following requirements:

1. **Docker** [Linux](https://docs.docker.com/desktop/setup/install/linux/ubuntu/) | [Windows](https://docs.docker.com/desktop/setup/install/windows-install/) | [Mac](https://docs.docker.com/desktop/setup/install/mac-install/)
2. **Python 3.8+** with `pip`.
3. **Git**: For repository management.

---

## Project Structure
A successful Caravel project requires a specific directory layout for the automated tools to function:

| Directory | Description |
| :--- | :--- |
| `openlane/` | Configuration files for hardening macros and the wrapper. |
| `verilog/rtl/` | Source Verilog code for the project. |
| `verilog/gl/` | Gate-level netlists (generated after hardening). |
| `verilog/dv/` | Design Verification (cocotb and Verilog testbenches). |
| `gds/` | Final GDSII binary files for fabrication. |
| `lef/` | Library Exchange Format files for the macros. |

---

## Starting Your Project

### 1. Repository Setup
Create a new repository based on the `caravel_user_project` template and clone it to your local machine:

```bash
git clone https://github.com/fidel-makatia/senseedge-asic.git
pip install chipfoundry-cli
cd senseedge-asic
```

### 2. Project Initialization

> [!IMPORTANT]
> Run this first! Initialize your project configuration:

```bash
cf init
```

This creates `.cf/project.json` with project metadata. **This must be run before any other commands** (`cf setup`, `cf gpio-config`, `cf harden`, `cf precheck`, `cf verify`).

### 3. Environment Setup
Install the ChipFoundry CLI tool and set up the local environment (PDKs, OpenLane, and Caravel lite):

```bash
cf setup
```

The `cf setup` command installs:

- Caravel Lite: The Caravel SoC template.
- Management Core: RISC-V management area required for simulation.
- OpenLane: The RTL-to-GDS hardening flow.
- PDK: Skywater 130nm process design kit.
- Timing Scripts: For Static Timing Analysis (STA).

---

## Development Flow

### Hardening the Design
Hardening is the process of synthesizing your RTL and performing Place & Route (P&R) to create a GDSII layout.

#### Macro Hardening
Create a subdirectory for each custom macro under `openlane/` containing your `config.tcl`.

```bash
cf harden --list         # List detected configurations
cf harden <macro_name>   # Harden a specific macro
```

#### Integration
Instantiate your module(s) in `verilog/rtl/user_project_wrapper.v`.

Update `openlane/user_project_wrapper/config.json` environment variables (`VERILOG_FILES_BLACKBOX`, `EXTRA_LEFS`, `EXTRA_GDS_FILES`) to point to your new macros.

#### Wrapper Hardening
Finalize the top-level user project:

```bash
cf harden user_project_wrapper
```

### Verification

#### 1. Simulation
We use cocotb for functional verification. Ensure your file lists are updated in `verilog/includes/`.

**Configure GPIO settings first (required before verification):**

```bash
cf gpio-config
```

This interactive command will:
- Configure all GPIO pins interactively
- Automatically update `verilog/rtl/user_defines.v`
- Automatically run `gen_gpio_defaults.py` to generate GPIO defaults for simulation

GPIO configuration is required before running any verification tests.

Run RTL Simulation:

```bash
cf verify <test_name>
```

Run Gate-Level (GL) Simulation:

```bash
cf verify <test_name> --sim gl
```

Run all tests:

```bash
cf verify --all
```

#### 2. Static Timing Analysis (STA)
Verify that your design meets timing constraints using OpenSTA:

```bash
make extract-parasitics
make create-spef-mapping
make caravel-sta
```

> [!NOTE]
> Run `make setup-timing-scripts` if you need to update the STA environment.

---

## GPIO Configuration
Configure the power-on default configuration for each GPIO using the interactive CLI tool.

**Use the GPIO configuration command:**
```bash
cf gpio-config
```

This command will:
- Present an interactive form for configuring GPIO pins 5-37 (GPIO 0-4 are fixed system pins)
- Show available GPIO modes with descriptions
- Allow selection by number, partial key, or full mode name
- Save configuration to `.cf/project.json` (as hex values)
- Automatically update `verilog/rtl/user_defines.v` with the new configuration
- Automatically run `gen_gpio_defaults.py` to generate GPIO defaults for simulation (if Caravel is installed)

**GPIO Pin Information:**
- GPIO[0] to GPIO[4]: Preset system pins (do not change).
- GPIO[5] to GPIO[37]: User-configurable pins.

**Available GPIO Modes:**
- Management modes: `mgmt_input_nopull`, `mgmt_input_pulldown`, `mgmt_input_pullup`, `mgmt_output`, `mgmt_bidirectional`, `mgmt_analog`
- User modes: `user_input_nopull`, `user_input_pulldown`, `user_input_pullup`, `user_output`, `user_bidirectional`, `user_output_monitored`, `user_analog`

> [!NOTE]
> GPIO configuration is required before running `cf precheck` or `cf verify`. Invalid modes cannot be saved - all GPIOs must have valid configurations.

---

## Local Precheck
Before submitting your design for fabrication, run the local precheck to ensure it complies with all shuttle requirements:

> [!IMPORTANT]
> GPIO configuration is required before running precheck. Make sure you've run `cf gpio-config` first.

```bash
cf precheck
```

You can also run specific checks or disable LVS:

```bash
cf precheck --disable-lvs                    # Skip LVS check
cf precheck --checks license --checks makefile  # Run specific checks only
```

---

## Checklist for Shuttle Submission
- [ ] Top-level macro is named user_project_wrapper.
- [ ] Full Chip Simulation passes for both RTL and GL.
- [ ] Hardened Macros are LVS and DRC clean.
- [ ] user_project_wrapper matches the required pin order/template.
- [ ] Design passes the local cf precheck.
- [ ] Documentation (this README) is updated with project-specific details.

---

## References

1. Case Western Reserve University Bearing Data Center — https://engineering.case.edu/bearingdatacenter
2. R. B. Randall, *Vibration-based Condition Monitoring*, Wiley, 2011
3. W. Zhang et al., "A New Deep Learning Model for Fault Diagnosis with Good Anti-Noise and Domain Adaptation Ability on Raw Vibration Signals," *Sensors*, 2017
4. OpenLane RTL-to-GDSII Flow — https://github.com/The-OpenROAD-Project/OpenLane
5. SkyWater SKY130 PDK — https://github.com/google/skywater-pdk

---

## License

All hardware, RTL, firmware, and documentation released under **Apache 2.0**.
Mechanical designs released under **CERN-OHL-P v2** (permissive).

---

**Author:** Fidel Makatia
**Contest:** ChipFoundry Reference Application Design Contest 2026

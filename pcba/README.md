# SenseEdge PCBA Design

Compact sensor node PCB for the SenseEdge predictive maintenance system.

## Board Specifications

| Parameter | Value |
|---|---|
| Dimensions | 45mm x 35mm |
| Layers | 2-layer FR4, 1.6mm |
| Copper | 1 oz (35um) |
| Surface Finish | HASL or ENIG |
| Min Trace/Space | 0.15mm / 0.15mm |
| Min Via | 0.3mm drill, 0.6mm pad |
| Mounting | 4x M3 through-holes at corners |

## Component Reference Designators

| Ref | Part | Package | Description |
|---|---|---|---|
| U3 | Caravel ASIC (SenseEdge) | QFN-64 | FFT + NN inference engine |
| U2 | ADXL326BCPZ-RL7 | 16-LFCSP (4x4mm) | 3-axis ±16g accelerometer |
| J1 | MCP3201T-CI/SN | SOIC-8 | 12-bit SPI ADC |
| U1 | ESP32-C3-MINI-1-N4 | Module | WiFi/BLE wireless |
| U4 | W25Q32JVSSIQ | SOIC-8 | 32 Mbit SPI flash |
| Y1 | ECS-2520S33-200-FN-TR | 2.5x2.0mm | 20 MHz CMOS oscillator |
| U6 | AP2112K-3.3TRG1 | SOT-23-5 | 3.3V LDO regulator |
| U5 | MIC5232-1.8YD5 | SOT-23-5 | 1.8V LDO regulator |
| J2 | UJ20-C-H-G-SMT-1-P16-TR | SMD | USB-C connector (power) |
| J3 | 22-28-4023 | Through-hole | 2-pin header (auxiliary) |
| SW1 | 1825910-6 | 6x6mm | Tactile reset button |
| D1 | APTF1616LSEKJ3ZGKQBC | 1616 | RGB LED (status/alarm) |

## Signal Path

```
ADXL326 (U2) → [Analog] → MCP3201 (J1) → [SPI] → Caravel ASIC (U3)
  Vibration              12-bit ADC            GPIO[0:2]
                                                  ↓
                                            ESP32-C3 (U1) → WiFi/BLE
                                              GPIO[5:6] (UART)
```

## Power

- **USB-C** (J2) provides 5V input
- **AP2112K-3.3** (U6) regulates to 3.3V for ADC, accelerometer, ESP32, and Caravel VDDIO
- **MIC5232-1.8** (U5) regulates to 1.8V for Caravel core VDD

## Clock

- **ECS-2520S33-200-FN-TR** (Y1) — 20 MHz CMOS oscillator → U3 clock input (no load caps needed)

## Communication

- **UART** (GPIO[5]/GPIO[6]) connects to **ESP32-C3** (U1) for WiFi/BLE
- **Alarm GPIO** (GPIO[3]) drives RGB LED (D1) red channel
- **Status GPIO** (GPIO[4]) drives RGB LED (D1) green channel

## Connectors

| Ref | Part | Purpose |
|---|---|---|
| J2 | UJ20-C-H-G-SMT-1-P16-TR | USB-C power input (5V) |
| J3 | 22-28-4023 | 2-pin auxiliary header |

## Assembly Notes

1. Solder QFN-64 (U3 — Caravel ASIC) first using reflow or hot air
2. Place remaining SMD components (U1, U2, U4, U5, U6, J1)
3. Reflow all SMD components
4. Hand-solder through-hole connectors (J2 USB-C, J3 header) and SW1
5. Clean flux residue before conformal coating

## Estimated BOM Cost

| Quantity | Total BOM |
|---|---|
| 1 unit | ~$24.20 |
| 100 units | ~$13.15 |

(Excludes Caravel ASIC — sponsored by ChipFoundry)

## Output Files

| Directory | Contents |
|---|---|
| `gerbers/` | Gerber files for PCB fabrication |
| `drills/` | NC drill files (plated, non-plated, slot holes) |
| `assembly/` | Pick & place files, assembly drawings |

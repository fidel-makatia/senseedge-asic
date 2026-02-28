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

## Schematic Overview

### Power
- **USB-C** (J1) provides 5V input
- **AP2112K-3.3** (U5) regulates to 3.3V for VDDIO, ADC, accelerometer, and ESP32
- **AP2112K-1.8** (U6) regulates to 1.8V for Caravel core VDD
- **M12 connector** (J2) provides alternative 12-24V industrial power input

### Signal Path
```
ADXL1002 (U3) → [Analog] → MCP3201 (U2) → [SPI] → Caravel ASIC (U1)
    Vibration          12-bit ADC              GPIO[0:2]
```

### Communication
- **UART** (GPIO 5/6) connects to **ESP32-C3** (U4) for WiFi/BLE
- **Alarm GPIO** (GPIO 3) drives RGB LED (D1) directly

### Clock
- **25 MHz crystal** (Y1) with 22pF load capacitors provides Caravel system clock

## Component Selection Rationale

| Component | Why |
|---|---|
| MCP3201 | Simple SPI protocol matching spi_adc_if.v, 12-bit, 100kSPS, $1.85 |
| ADXL1002 | Wide bandwidth (11kHz), high g-range (+/-50g), analog output |
| ESP32-C3-MINI | Smallest WiFi+BLE module, UART interface, $1.90 |
| AP2112K | Ultra-low dropout, 600mA, SOT-23-5, $0.35 |

## Assembly Notes

1. Solder QFN-64 (U1) first using reflow or hot air
2. Place remaining SMD components
3. Hand-solder through-hole connectors (J1, J2) last
4. Clean flux residue before conformal coating

## Estimated BOM Cost

| Quantity | Total BOM |
|---|---|
| 1 unit | ~$23.50 |
| 100 units | ~$13.00 |

(Excludes Caravel ASIC — sponsored by ChipFoundry)

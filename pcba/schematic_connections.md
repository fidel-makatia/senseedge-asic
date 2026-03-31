# SenseEdge PCB — Complete Schematic Connections v1.0

## U1 — Caravel ASIC (QFN-64)

| Pin | Signal | Net | Connects To |
|-----|--------|-----|-------------|
| 1 | resetb | RESETB | R7 (10kΩ to 3V3) + C11 (100nF to GND) + SW1 reset button |
| 2 | clock | CLK_20M | Y1 pin 3 (OUT) — ECS-2520S33-200-FN-TR, 20 MHz CMOS oscillator |
| 3 | flash_csb | FLASH_CSB | U7 (W25Q32) pin 1 |
| 4 | flash_clk | FLASH_CLK | U7 pin 6 |
| 5 | flash_io0 | FLASH_IO0 | U7 pin 5 (DI) |
| 6 | flash_io1 | FLASH_IO1 | U7 pin 2 (DO) |
| 7 | gpio | MGMT_GPIO | R8 (10kΩ to 3V3) — management GPIO |
| 8 | vssa | GND | Ground plane |
| 9 | vssd | GND | Ground plane |
| 10 | vssio | GND | Ground plane |
| 11 | vdda | 3V3 | 3.3V rail + C12 (100nF to GND) |
| 12 | vccd | 1V8 | 1.8V rail + C13 (100nF to GND) |
| 13 | mprj_io[0] | SPI_MISO | U2 pin 7 (DOUT) |
| 14 | vddio | 3V3 | 3.3V rail + C14 (100nF to GND) |
| 15 | mprj_io[1] | SPI_CLK | U2 pin 8 (CLK) |
| 16 | mprj_io[37] | NC | No connect |
| 17 | mprj_io[2] | SPI_CS_N | U2 pin 6 (~CS) |
| 18 | mprj_io[36] | NC | No connect |
| 19 | mprj_io[3] | ALARM | R1 (330Ω) → D1 red anode |
| 20 | mprj_io[35] | NC | No connect |
| 21 | mprj_io[4] | STATUS_LED | R2 (330Ω) → D1 green anode |
| 22 | mprj_io[34] | NC | No connect |
| 23 | mprj_io[5] | UART_TX | U4 pin 20 (RX) |
| 24 | mprj_io[33] | NC | No connect |
| 25 | mprj_io[6] | UART_RX | U4 pin 21 (TX) |
| 26 | mprj_io[32] | NC | No connect |
| 27 | vssa1_2 | GND | Ground plane |
| 28 | vssd2 | GND | Ground plane |
| 29 | vssd1 | GND | Ground plane |
| 30 | vdda2 | 3V3 | 3.3V rail + C15 (100nF to GND) |
| 31 | vdda1_2 | 3V3 | 3.3V rail |
| 32 | mprj_io[31] | NC | No connect |
| 33 | mprj_io[7] | NC | No connect |
| 34 | mprj_io[30] | NC | No connect |
| 35 | mprj_io[8] | NC | No connect |
| 36 | mprj_io[29] | NC | No connect |
| 37 | mprj_io[9] | NC | No connect |
| 38 | mprj_io[28] | NC | No connect |
| 39 | mprj_io[10] | NC | No connect |
| 40 | mprj_io[27] | NC | No connect |
| 41 | mprj_io[11] | NC | No connect |
| 42 | mprj_io[26] | NC | No connect |
| 43 | mprj_io[12] | NC | No connect |
| 44 | mprj_io[25] | NC | No connect |
| 45 | vdda1 | 3V3 | 3.3V rail + C16 (100nF to GND) |
| 46 | vssa2 | GND | Ground plane |
| 47 | mprj_io[13] | NC | No connect |
| 48 | vddio_2 | 3V3 | 3.3V rail + C17 (100nF to GND) |
| 49 | vccd1 | 1V8 | 1.8V rail + C18 (100nF to GND) |
| 50 | vccd2 | 1V8 | 1.8V rail + C19 (100nF to GND) |
| 51 | mprj_io[14] | NC | No connect |
| 52 | mprj_io[24] | NC | No connect |
| 53 | vssio_2 | GND | Ground plane |
| 54 | vssa1 | GND | Ground plane |
| 55 | mprj_io[23] | NC | No connect |
| 56 | mprj_io[22] | NC | No connect |
| 57 | mprj_io[21] | NC | No connect |
| 58 | mprj_io[20] | NC | No connect |
| 59 | mprj_io[19] | NC | No connect |
| 60 | mprj_io[18] | NC | No connect |
| 61 | mprj_io[17] | NC | No connect |
| 62 | mprj_io[16] | NC | No connect |
| 63 | mprj_io[15] | NC | No connect |
| 65 | EPAD | GND | Thermal pad → ground plane (multiple vias) |

---

## U2 — MCP3201 (12-bit SPI ADC, SOIC-8)

| Pin | Signal | Net | Connects To |
|-----|--------|-----|-------------|
| 1 | VDD | 3V3 | 3.3V rail + C3 (100nF to GND) |
| 2 | VREF | 3V3 | 3.3V rail (reference voltage) |
| 3 | IN+ | VIB_ANALOG | U3 pin 12 (OUT) via R3 (100Ω series) + C4 (100nF to GND, anti-alias) |
| 4 | IN- | GND | Ground plane |
| 5 | VSS | GND | Ground plane |
| 6 | ~CS | SPI_CS_N | U1 pin 17 (mprj_io[2]) |
| 7 | DOUT | SPI_MISO | U1 pin 13 (mprj_io[0]) |
| 8 | CLK | SPI_CLK | U1 pin 15 (mprj_io[1]) |

---

## U3 — ADXL326BCPZ-RL7 (3-Axis ±16g Accelerometer, 16-LFCSP 4x4mm)

| Pin | Signal | Net | Connects To |
|-----|--------|-----|-------------|
| 1 | NC | NC | Leave floating (or GND) |
| 2 | ST | GND | Ground (self-test disabled) |
| 3 | COM | GND | Ground plane |
| 4 | NC | NC | Leave floating |
| 5 | COM | GND | Ground plane |
| 6 | COM | GND | Ground plane |
| 7 | COM | GND | Ground plane |
| 8 | ZOUT | Z_OUT | CZ (10nF to GND) — 500 Hz BW. Route to test pad |
| 9 | NC | NC | Leave floating (or GND) |
| 10 | YOUT | Y_OUT | CY (3.3nF to GND) — 1.5 kHz BW. Route to test pad |
| 11 | NC | NC | Leave floating |
| 12 | XOUT | VIB_ANALOG | CX (3.3nF to GND) + R3 (100Ω) → U2 pin 3 (MCP3201 IN+) |
| 13 | NC | NC | Leave floating |
| 14 | VS | 3V3 | 3.3V rail + C5 (1µF to GND) + C6 (100nF to GND) |
| 15 | VS | 3V3 | 3.3V rail (tie to pin 14) |
| 16 | NC | NC | Leave floating |
| EP | Exposed pad | GND | Solder to GND pad (mechanical only, not internally connected) |

---

## U4 — ESP32-C3-MINI-1 (WiFi/BLE Module)

| Pin | Signal | Net | Connects To |
|-----|--------|-----|-------------|
| 1 | GND | GND | Ground plane |
| 2 | 3V3 | 3V3 | 3.3V rail + C7 (10µF to GND) + C8 (100nF to GND) |
| 3 | EN | EN | R4 (10kΩ to 3V3) + C9 (1µF to GND) — RC delay reset |
| 4-14 | Various | NC | No connect (unused GPIOs) |
| 15 | GPIO9 | BOOT | R5 (10kΩ to 3V3) — boot mode select |
| 16-19 | Various | NC | No connect |
| 20 | RX (GPIO20) | UART_TX | U1 pin 23 (mprj_io[5]) |
| 21 | TX (GPIO21) | UART_RX | U1 pin 25 (mprj_io[6]) |
| 39 | GND (pad) | GND | Ground plane |

---

## U5 — AP2112K-3.3 (3.3V LDO, SOT-23-5)

| Pin | Signal | Net | Connects To |
|-----|--------|-----|-------------|
| 1 | VIN | VBUS_5V | J1 VBUS + C20 (10µF to GND) |
| 2 | GND | GND | Ground plane |
| 3 | EN | VBUS_5V | Tied to VIN (always enabled) |
| 4 | NC | NC | No connect |
| 5 | VOUT | 3V3 | 3.3V rail + C21 (10µF to GND) + C22 (100nF to GND) |

---

## U6 — AP2112K-1.8 (1.8V LDO, SOT-23-5)

| Pin | Signal | Net | Connects To |
|-----|--------|-----|-------------|
| 1 | VIN | 3V3 | 3.3V rail |
| 2 | GND | GND | Ground plane |
| 3 | EN | 3V3 | Tied to VIN (always enabled) |
| 4 | NC | NC | No connect |
| 5 | VOUT | 1V8 | 1.8V rail + C23 (10µF to GND) + C24 (100nF to GND) |

---

## U7 — W25Q32JVSS (32Mbit SPI Flash, SOIC-8)

| Pin | Signal | Net | Connects To |
|-----|--------|-----|-------------|
| 1 | ~CS | FLASH_CSB | U1 pin 3 |
| 2 | DO | FLASH_IO1 | U1 pin 6 |
| 3 | ~WP | 3V3 | Pulled high (write protect disabled) |
| 4 | GND | GND | Ground plane |
| 5 | DI | FLASH_IO0 | U1 pin 5 |
| 6 | CLK | FLASH_CLK | U1 pin 4 |
| 7 | ~HOLD | 3V3 | Pulled high (hold disabled) |
| 8 | VCC | 3V3 | 3.3V rail + C25 (100nF to GND) |

---

## Y1 — ECS-2520S33-200-FN-TR (20 MHz CMOS Oscillator, 2.5x2.0mm)

| Pin | Name | Net | Connects To |
|-----|------|-----|-------------|
| 1 | EN | 3V3 | Tied high (always enabled) |
| 2 | GND | GND | Ground plane |
| 3 | OUT | CLK_20M | U1 pin 2 (clock) — series 33Ω R11 recommended |
| 4 | VDD | 3V3 | 3.3V rail + C1 (100nF to GND) |

---

## D1 — RGB LED (Common Cathode)

| Pin | Net | Connects To |
|-----|-----|-------------|
| 1 (Red anode) | ALARM_LED | R1 (330Ω) ← U1 pin 19 (GPIO[3]) |
| 2 (Green anode) | STATUS_LED | R2 (330Ω) ← U1 pin 21 (GPIO[4]) |
| 3 (Blue anode) | NC | No connect |
| 4 (Cathode) | GND | Ground plane |

---

## J1 — USB-C Connector (Power Only)

| Pin | Net | Connects To |
|-----|-----|-------------|
| A1/B12 | GND | Ground plane |
| A4/B9 | VBUS_5V | U5 pin 1 (VIN) + C20 (10µF) |
| A5 | CC1 | R9 (5.1kΩ to GND) — USB-C detection |
| B5 | CC2 | R10 (5.1kΩ to GND) — USB-C detection |
| Shell | GND | Ground plane |

---

## J2 — M12-A 4-Pin Connector (Industrial)

| Pin | Net | Purpose |
|-----|-----|---------|
| 1 | VIN_12-24V | Optional industrial power input (to U5 via D2 protection diode) |
| 2 | GND | Ground |
| 3 | ALARM_OUT | Buffered alarm output (from GPIO[3] via Q1 MOSFET) |
| 4 | GND | Ground |

---

## SW1 — Reset Button (Tactile Switch)

| Pin | Net | Connects To |
|-----|-----|-------------|
| 1 | RESETB | U1 pin 1 + R7 (10kΩ to 3V3) |
| 2 | GND | Ground plane |

---

## Full BOM

| Ref | Value | Package | Qty | Purpose |
|-----|-------|---------|-----|---------|
| U1 | Caravel ASIC | QFN-64 | 1 | FFT + NN inference |
| U2 | MCP3201 | SOIC-8 | 1 | 12-bit SPI ADC |
| U3 | ADXL326BCPZ-RL7 | 16-LFCSP | 1 | 3-axis ±16g accelerometer |
| U4 | ESP32-C3-MINI-1 | Module | 1 | WiFi/BLE |
| U5 | AP2112K-3.3 | SOT-23-5 | 1 | 3.3V LDO |
| U6 | AP2112K-1.8 | SOT-23-5 | 1 | 1.8V LDO |
| U7 | W25Q32JVSS | SOIC-8 | 1 | SPI Flash (firmware) |
| Y1 | ECS-2520S33-200-FN-TR | 2.5x2.0mm | 1 | 20 MHz CMOS oscillator |
| D1 | RGB LED | 1206 | 1 | Status indicator |
| D2 | SS34 | SMA | 1 | Reverse polarity protection |
| Q1 | 2N7002 | SOT-23 | 1 | Alarm output buffer |
| SW1 | Tactile | 6x6mm | 1 | Reset button |
| J1 | USB-C | SMD | 1 | Power input |
| J2 | M12-A 4-pin | Panel mount | 1 | Industrial connector |
| R1,R2 | 330Ω | 0402 | 2 | LED current limiting |
| R3 | 100Ω | 0402 | 1 | ADC input series |
| R4,R5,R7,R8 | 10kΩ | 0402 | 4 | Pull-ups |
| R9,R10 | 5.1kΩ | 0402 | 2 | USB-C CC detection |
| R11 | 33Ω | 0402 | 1 | Clock series termination |
| C1,C3,C6,C8,C12-C19,C22,C24,C25 | 100nF | 0402 | 16 | Decoupling |
| CX,CY | 3.3nF | 0402 | 2 | ADXL326 X/Y bandwidth (1.5 kHz) |
| CZ | 10nF | 0402 | 1 | ADXL326 Z bandwidth (500 Hz) |
| C5 | 1µF | 0402 | 1 | ADXL326 bulk decoupling |
| C9 | 1µF | 0402 | 1 | ESP32 EN delay |
| C7,C20,C21,C23 | 10µF | 0805 | 4 | Bulk decoupling |
| C11 | 100nF | 0402 | 1 | Reset filter |

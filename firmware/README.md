# SenseEdge Firmware

RISC-V firmware for the Caravel management core that controls the SenseEdge hardware pipeline.

## Files

| File | Description |
|---|---|
| `senseedge_fw.c` | Main firmware: GPIO init, weight loading, pipeline control, UART output |
| `senseedge_regs.h` | Register map definitions matching `wb_interface.v` |
| `nn_weights.h` | Pre-trained INT8 neural network weights (212 parameters) |

## Firmware Flow

1. **Configure GPIOs** for SPI ADC, alarm output, status LED, UART
2. **Load 212 NN weights** into hardware registers via Wishbone
3. **Set ADC clock divider** and alarm thresholds
4. **Enable hardware pipeline** (SPI → FFT → Features → NN → Alarm)
5. **Poll for results** and transmit classification via UART to ESP32

## UART Protocol

Results are sent at 115200 baud:
```
CLASS:HEALTHY CONF:230 ALARM:0
CLASS:BEARING_WEAR CONF:185 ALARM:0
CLASS:BEARING_WEAR CONF:192 ALARM:1
*** ALARM: Fault detected! ***
```

## Building

The firmware is compiled using the Caravel RISC-V toolchain as part of the cocotb test flow:
```bash
cf verify senseedge_wb
```

## Updating Weights

To load new weights from a retrained model:
```bash
cd ../ml
python3 train_senseedge.py
python3 export_weights.py
```
This regenerates `nn_weights.h` with updated INT8 weights.

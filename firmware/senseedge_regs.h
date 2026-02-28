// SPDX-License-Identifier: Apache-2.0
// SenseEdge Register Map Definitions
// Matches wb_interface.v register layout

#ifndef SENSEEDGE_REGS_H
#define SENSEEDGE_REGS_H

// Caravel user project Wishbone base address
#define SE_BASE             0x30000000

// Control and status registers
#define SE_CTRL             (SE_BASE + 0x00)  // R/W: [0]=enable
#define SE_STATUS           (SE_BASE + 0x04)  // R:   [0]=enable [1]=fft_busy [2]=nn_busy [3]=fe_busy [4]=alarm
#define SE_CLASS_RESULT     (SE_BASE + 0x08)  // R:   [1:0]=class_id [9:2]=confidence
#define SE_ALARM_CFG        (SE_BASE + 0x0C)  // R/W: [7:0]=threshold [11:8]=consecutive_faults
#define SE_FFT_DATA         (SE_BASE + 0x10)  // R:   16-bit FFT magnitude (auto-increment)
#define SE_FEATURE_DATA     (SE_BASE + 0x14)  // R:   8-bit feature value (auto-increment)
#define SE_IRQ_FLAGS        (SE_BASE + 0x18)  // R/W: [0]=class_done [1]=alarm_irq
#define SE_CLK_DIV          (SE_BASE + 0x1C)  // R/W: [15:0]=ADC clock divider
#define SE_NN_WEIGHTS       (SE_BASE + 0x20)  // W:   NN weight write (addr in [15:8], data in [7:0])

// Status register bit positions
#define STATUS_ENABLE       (1 << 0)
#define STATUS_FFT_BUSY     (1 << 1)
#define STATUS_NN_BUSY      (1 << 2)
#define STATUS_FE_BUSY      (1 << 3)
#define STATUS_ALARM        (1 << 4)

// IRQ flag bit positions
#define IRQ_CLASS_DONE      (1 << 0)
#define IRQ_ALARM           (1 << 1)

// Classification classes
#define CLASS_HEALTHY        0
#define CLASS_BEARING_WEAR   1
#define CLASS_IMBALANCE      2
#define CLASS_MISALIGNMENT   3

// Extract fields from CLASS_RESULT register
#define GET_CLASS_ID(reg)    ((reg) & 0x3)
#define GET_CONFIDENCE(reg)  (((reg) >> 2) & 0xFF)

// Pack alarm config: threshold in [7:0], fault count in [11:8]
#define ALARM_CFG(threshold, faults)  (((faults) << 8) | ((threshold) & 0xFF))

// Pack NN weight write: address in [15:8], data in [7:0]
#define NN_WEIGHT(addr, data)  (((addr) << 8) | ((data) & 0xFF))

// NN weight memory layout
#define NN_L1_WEIGHTS_START   0    // Layer 1 weights: [0..127] (16 neurons x 8 inputs)
#define NN_L1_BIASES_START    128  // Layer 1 biases:  [128..143]
#define NN_L2_WEIGHTS_START   144  // Layer 2 weights: [144..207] (4 neurons x 16 inputs)
#define NN_L2_BIASES_START    208  // Layer 2 biases:  [208..211]
#define NN_TOTAL_PARAMS       212

#endif // SENSEEDGE_REGS_H

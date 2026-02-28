// SPDX-License-Identifier: Apache-2.0
// SenseEdge - Caravel Firmware for Wishbone Integration Test
// Configures the SenseEdge ASIC: loads NN weights, sets alarm thresholds,
// enables the acquisition pipeline, and reads classification results.

#include <firmware_apis.h>

// SenseEdge register offsets (from user project base address)
#define SE_CTRL          0x00
#define SE_STATUS        0x04
#define SE_CLASS_RESULT  0x08
#define SE_ALARM_CFG     0x0C
#define SE_FFT_DATA      0x10
#define SE_FEATURE_DATA  0x14
#define SE_IRQ_FLAGS     0x18
#define SE_CLK_DIV       0x1C
#define SE_NN_WEIGHTS    0x20

void main() {
    // Enable management GPIO as output indicator
    ManagmentGpio_outputEnable();
    ManagmentGpio_write(0);
    enableHkSpi(0);

    // Configure GPIOs:
    // io[0] = SPI MISO (input from ADC)
    // io[1] = SPI CLK  (output to ADC)
    // io[2] = SPI CS_N (output to ADC)
    // io[3] = Alarm    (output)
    // io[4] = Status LED (output)
    // io[30-37] = unused outputs (for LA visibility)
    GPIOs_configure(0, GPIO_MODE_USER_STD_INPUT_NOPULL);   // MISO
    GPIOs_configure(1, GPIO_MODE_USER_STD_OUTPUT);         // SPI CLK
    GPIOs_configure(2, GPIO_MODE_USER_STD_OUTPUT);         // SPI CS
    GPIOs_configure(3, GPIO_MODE_USER_STD_OUTPUT);         // Alarm
    GPIOs_configure(4, GPIO_MODE_USER_STD_OUTPUT);         // Status LED
    GPIOs_configure(5, GPIO_MODE_USER_STD_OUTPUT);         // UART TX
    GPIOs_configure(6, GPIO_MODE_USER_STD_INPUT_NOPULL);   // UART RX

    GPIOs_loadConfigs();

    // Enable Wishbone user interface
    User_enableIF();

    // Signal: configuration phase started
    ManagmentGpio_write(1);

    // --- Phase 1: Load NN weights ---
    // Simple identity-like weights for testing
    int i;

    // Layer 1 weights (128 bytes): zero all
    for (i = 0; i < 128; i++)
        USER_writeWord(0, SE_NN_WEIGHTS + i);

    // Set diagonal weights: neuron K responds to input K
    USER_writeWord(127, SE_NN_WEIGHTS + 0);    // w[0][0]
    USER_writeWord(127, SE_NN_WEIGHTS + 9);    // w[1][1]
    USER_writeWord(127, SE_NN_WEIGHTS + 18);   // w[2][2]
    USER_writeWord(127, SE_NN_WEIGHTS + 27);   // w[3][3]

    // Layer 1 biases (16 bytes): zero
    for (i = 128; i < 144; i++)
        USER_writeWord(0, SE_NN_WEIGHTS + i);

    // Layer 2 weights (64 bytes): zero all
    for (i = 144; i < 208; i++)
        USER_writeWord(0, SE_NN_WEIGHTS + i);

    // Class K responds to hidden neuron K
    USER_writeWord(127, SE_NN_WEIGHTS + 144);  // class0 <- hidden0
    USER_writeWord(127, SE_NN_WEIGHTS + 161);  // class1 <- hidden1
    USER_writeWord(127, SE_NN_WEIGHTS + 178);  // class2 <- hidden2
    USER_writeWord(127, SE_NN_WEIGHTS + 195);  // class3 <- hidden3

    // Layer 2 biases (4 bytes): zero
    for (i = 208; i < 212; i++)
        USER_writeWord(0, SE_NN_WEIGHTS + i);

    // --- Phase 2: Configure system ---
    // Set clock divider for SPI (fast for simulation)
    USER_writeWord(4, SE_CLK_DIV);

    // Set alarm config: threshold=100, consecutive_faults=3
    USER_writeWord(0x00000364, SE_ALARM_CFG);

    // Signal: weights loaded
    ManagmentGpio_write(0);

    // --- Phase 3: Enable acquisition ---
    USER_writeWord(1, SE_CTRL);

    // Wait for pipeline to complete
    // Poll status register for completion (nn_busy goes low)
    int timeout = 100000;
    while (timeout > 0) {
        unsigned int status = USER_readWord(SE_STATUS);
        // bits: [0]=enable [1]=fft_busy [2]=nn_busy [3]=fe_busy [4]=alarm
        if ((status & 0x06) == 0 && (status & 0x01)) {
            // FFT and NN not busy, still enabled
            // Check if we've had at least one classification
            unsigned int irq_flags = USER_readWord(SE_IRQ_FLAGS);
            if (irq_flags & 0x01) break;  // classification_done flag
        }
        timeout--;
    }

    // --- Phase 4: Read results ---
    unsigned int class_result = USER_readWord(SE_CLASS_RESULT);
    unsigned int class_id   = class_result & 0x03;
    unsigned int confidence = (class_result >> 2) & 0xFF;

    // Signal result via management GPIO
    // GPIO high = test complete
    ManagmentGpio_write(1);

    // Read and report FFT bins (first 4)
    USER_writeWord(0, SE_FFT_DATA); // Reset read pointer
    for (i = 0; i < 4; i++) {
        unsigned int fft_val = USER_readWord(SE_FFT_DATA);
        // Value available for cocotb to read via LA
    }

    // Done - hold
    while(1);
}

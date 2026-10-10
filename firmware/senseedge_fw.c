// SPDX-License-Identifier: Apache-2.0
// SenseEdge Firmware for Caravel RISC-V Management Core
// Corrected for the RTL of branch fix/audit-defects: word-wide weight load with read-back,
// absolute-address user-space access, hardware UART on pad 6, no division (rv32i).
//
// This firmware initializes the SenseEdge predictive maintenance pipeline:
//   1. Configure GPIO pins for SPI ADC, alarm, UART
//   2. Load pre-trained INT8 neural network weights
//   3. Set ADC sample rate and alarm thresholds
//   4. Enable the hardware pipeline
//   5. Poll for classification results and transmit via the hardware UART (pad 6)

#include <firmware_apis.h>
#include "senseedge_regs.h"
#include "nn_weights.h"

// ---------- Configuration ----------

#define ADC_CLK_DIVIDER     249     // divider D: f_s = f_clk / (35 (D+1)); 20 MHz, D=249 -> 2.29 kS/s
#define ALARM_THRESHOLD     150     // Confidence threshold for fault alarm
#define ALARM_FAULT_COUNT   3       // Consecutive faults before alarm triggers

// ---------- UART ----------
// The management core's hardware UART: TX on pad 6, RX on pad 5 (unused here), at the
// core's default baud divider.  The first revision bit-banged pad 5 from software; in
// Caravel co-simulation its bit time varied from 7.5k to 11.8k cycles with the code path,
// because the core executes from SPI flash without an instruction cache, so no receiver
// could lock to it.  The board must connect the radio's RX to pad 6.

static void uart_send_byte(uint8_t byte)
{
    UART_sendChar((char)byte);
}

static void uart_send_string(const char *str)
{
    while (*str) {
        uart_send_byte((uint8_t)*str);
        str++;
    }
}

static void uart_send_hex(uint32_t val)
{
    int i;
    char hex_chars[] = "0123456789ABCDEF";

    uart_send_byte('0');
    uart_send_byte('x');
    for (i = 28; i >= 0; i -= 4) {
        uart_send_byte(hex_chars[(val >> i) & 0xF]);
    }
}

// No division or modulo: the management core is rv32i and the Caravel firmware build links
// without libgcc's __udivsi3/__umodsi3 (the original version failed to link).
static void uart_send_dec(uint32_t val)
{
    static const uint32_t pow10[10] = { 1000000000u, 100000000u, 10000000u, 1000000u, 100000u,
                                        10000u, 1000u, 100u, 10u, 1u };
    int i, started = 0;

    if (val == 0) {
        uart_send_byte('0');
        return;
    }
    for (i = 0; i < 10; i++) {
        unsigned int d = 0;
        while (val >= pow10[i]) { val -= pow10[i]; d++; }
        if (d || started) { uart_send_byte((uint8_t)('0' + d)); started = 1; }
    }
}

// ---------- Classification Result Names ----------

static const char *class_names[4] = {
    "HEALTHY",
    "BEARING_WEAR",
    "IMBALANCE",
    "MISALIGNMENT"
};

// ---------- Main Firmware ----------

void main(void)
{
    uint32_t i;
    uint32_t status;
    uint32_t result;
    uint32_t class_id;
    uint32_t confidence;
    uint32_t irq_flags;
    uint32_t cycle_count;
    uint32_t weight_errors;

    // --- Phase 1: GPIO Configuration ---
    ManagmentGpio_outputEnable();
    ManagmentGpio_write(0);
    enableHkSpi(0);

    // Configure SenseEdge GPIO pins
    // GPIO 0: SPI MISO (input from ADC)
    GPIOs_configure(0, GPIO_MODE_USER_STD_INPUT_NOPULL);
    // GPIO 1: SPI CLK (output to ADC)
    GPIOs_configure(1, GPIO_MODE_USER_STD_OUTPUT);
    // GPIO 2: SPI CS_N (output to ADC)
    GPIOs_configure(2, GPIO_MODE_USER_STD_OUTPUT);
    // GPIO 3: Alarm output (LED/buzzer)
    GPIOs_configure(3, GPIO_MODE_USER_STD_OUTPUT);
    // GPIO 4: Status LED
    GPIOs_configure(4, GPIO_MODE_USER_STD_OUTPUT);
    // GPIO 5: UART RX (management core), GPIO 6: UART TX (management core)
    GPIOs_configure(5, GPIO_MODE_MGMT_STD_INPUT_NOPULL);
    GPIOs_configure(6, GPIO_MODE_MGMT_STD_OUTPUT);

    GPIOs_loadConfigs();
    UART_enableTX(1);

    // Enable Wishbone user interface
    User_enableIF();

    // Signal: GPIO config complete
    ManagmentGpio_write(1);

    // --- Phase 2: Load Neural Network Weights ---
    // 53 word writes, four INT8 parameters per word, then read every word back.
    weight_errors = 0;
    for (i = 0; i < SE_NN_WEIGHT_WORDS; i++) {
        uint32_t word = NN_WEIGHT_WORD(all_weights[4 * i], all_weights[4 * i + 1],
                                       all_weights[4 * i + 2], all_weights[4 * i + 3]);
        SE_WRITE(SE_NN_WEIGHT_WORD(i), word);
    }
    for (i = 0; i < SE_NN_WEIGHT_WORDS; i++) {
        uint32_t word = NN_WEIGHT_WORD(all_weights[4 * i], all_weights[4 * i + 1],
                                       all_weights[4 * i + 2], all_weights[4 * i + 3]);
        if (SE_READ(SE_NN_WEIGHT_WORD(i)) != word)
            weight_errors++;
    }

    // Signal: weights loaded (and verified)
    ManagmentGpio_write(2);

    // --- Phase 3: Configure System ---
    // Set ADC clock divider for desired sample rate
    SE_WRITE(SE_CLK_DIV, ADC_CLK_DIVIDER);

    // Set alarm configuration: threshold and consecutive fault count
    SE_WRITE(SE_ALARM_CFG, ALARM_CFG(ALARM_THRESHOLD, ALARM_FAULT_COUNT));

    // Clear any pending IRQ flags
    SE_WRITE(SE_IRQ_FLAGS, 0x3);

    // --- Phase 4: Enable Pipeline ---
    SE_WRITE(SE_CTRL, 0x1);

    // Signal: system running
    ManagmentGpio_write(3);

    // Send startup message via UART
    uart_send_string("SenseEdge v1.1 Online\r\n");
    uart_send_string("WEIGHTS verified, word errors: ");
    uart_send_dec(weight_errors);
    uart_send_string("\r\n");
    uart_send_string("Monitoring vibration...\r\n");

    // --- Phase 5: Main Classification Loop ---
    while (1) {
        // Poll for classification done IRQ
        cycle_count = 0;
        do {
            irq_flags = SE_READ(SE_IRQ_FLAGS);
            cycle_count++;
            if (cycle_count > 1000000) {
                // Timeout — pipeline may be stuck
                uart_send_string("WARN: Pipeline timeout\r\n");
                break;
            }
        } while (!(irq_flags & IRQ_CLASS_DONE));

        // Clear IRQ flags
        SE_WRITE(SE_IRQ_FLAGS, irq_flags);

        // Read classification result
        result = SE_READ(SE_CLASS_RESULT);
        class_id = GET_CLASS_ID(result);
        confidence = GET_CONFIDENCE(result);

        // Read status (check alarm)
        status = SE_READ(SE_STATUS);

        // Transmit result via UART
        // Format: CLASS:<name> CONF:<value> ALARM:<0/1>\r\n
        uart_send_string("CLASS:");
        if (class_id < 4)
            uart_send_string(class_names[class_id]);
        else
            uart_send_string("UNKNOWN");

        uart_send_string(" CONF:");
        uart_send_dec(confidence);

        uart_send_string(" ALARM:");
        uart_send_byte((status & STATUS_ALARM) ? '1' : '0');
        uart_send_string("\r\n");

        // Check for alarm condition
        if (irq_flags & IRQ_ALARM) {
            uart_send_string("*** ALARM: Fault detected! ***\r\n");
            uart_send_string("Class: ");
            uart_send_string(class_names[class_id]);
            uart_send_string("\r\n");
        }

        // Signal: result available on management GPIO
        // Toggle between 3 and 4 to indicate new results
        ManagmentGpio_write(3 + (cycle_count & 1));
    }
}

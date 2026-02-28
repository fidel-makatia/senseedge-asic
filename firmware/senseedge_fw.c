// SPDX-License-Identifier: Apache-2.0
// SenseEdge Firmware for Caravel RISC-V Management Core
//
// This firmware initializes the SenseEdge predictive maintenance pipeline:
//   1. Configure GPIO pins for SPI ADC, alarm, UART
//   2. Load pre-trained INT8 neural network weights
//   3. Set ADC sample rate and alarm thresholds
//   4. Enable the hardware pipeline
//   5. Poll for classification results and transmit via UART

#include <firmware_apis.h>
#include "senseedge_regs.h"
#include "nn_weights.h"

// ---------- Configuration ----------

#define ADC_CLK_DIVIDER     250     // 25MHz / 250 = 100kHz SPI clock -> ~6.25 kSPS
#define ALARM_THRESHOLD     150     // Confidence threshold for fault alarm
#define ALARM_FAULT_COUNT   3       // Consecutive faults before alarm triggers

// UART bit-bang configuration (on GPIO 5)
#define UART_BAUD_DELAY     217     // ~115200 baud at 25 MHz (25M / 115200 = 217)
#define UART_GPIO           5

// ---------- UART Bit-Bang ----------

static void uart_delay(void)
{
    volatile int i;
    for (i = 0; i < UART_BAUD_DELAY; i++);
}

static void uart_send_byte(uint8_t byte)
{
    int bit;

    // Start bit (low)
    GPIOs_writeLow(UART_GPIO, 0);
    uart_delay();

    // Data bits (LSB first)
    for (bit = 0; bit < 8; bit++) {
        if (byte & (1 << bit))
            GPIOs_writeHigh(UART_GPIO, 1);
        else
            GPIOs_writeLow(UART_GPIO, 0);
        uart_delay();
    }

    // Stop bit (high)
    GPIOs_writeHigh(UART_GPIO, 1);
    uart_delay();
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

static void uart_send_dec(uint32_t val)
{
    char buf[10];
    int i = 0;

    if (val == 0) {
        uart_send_byte('0');
        return;
    }
    while (val > 0) {
        buf[i++] = '0' + (val % 10);
        val /= 10;
    }
    while (--i >= 0)
        uart_send_byte(buf[i]);
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
    // GPIO 5: UART TX (output to ESP32)
    GPIOs_configure(5, GPIO_MODE_USER_STD_OUTPUT);
    // GPIO 6: UART RX (input from ESP32)
    GPIOs_configure(6, GPIO_MODE_USER_STD_INPUT_NOPULL);

    GPIOs_loadConfigs();

    // Enable Wishbone user interface
    User_enableIF();

    // Signal: GPIO config complete
    ManagmentGpio_write(1);

    // --- Phase 2: Load Neural Network Weights ---
    // Write all 212 parameters to the NN weight registers
    // The hardware expects: address in [15:8], data in [7:0]
    for (i = 0; i < NN_TOTAL_PARAMS; i++) {
        USER_writeWord(NN_WEIGHT(i, (uint8_t)all_weights[i]), SE_NN_WEIGHTS);
    }

    // Signal: weights loaded
    ManagmentGpio_write(2);

    // --- Phase 3: Configure System ---
    // Set ADC clock divider for desired sample rate
    USER_writeWord(ADC_CLK_DIVIDER, SE_CLK_DIV);

    // Set alarm configuration: threshold and consecutive fault count
    USER_writeWord(ALARM_CFG(ALARM_THRESHOLD, ALARM_FAULT_COUNT), SE_ALARM_CFG);

    // Clear any pending IRQ flags
    USER_writeWord(0x3, SE_IRQ_FLAGS);

    // --- Phase 4: Enable Pipeline ---
    USER_writeWord(0x1, SE_CTRL);

    // Signal: system running
    ManagmentGpio_write(3);

    // Send startup message via UART
    uart_send_string("SenseEdge v1.0 Online\r\n");
    uart_send_string("Monitoring vibration...\r\n");

    // --- Phase 5: Main Classification Loop ---
    while (1) {
        // Poll for classification done IRQ
        cycle_count = 0;
        do {
            irq_flags = USER_readWord(SE_IRQ_FLAGS);
            cycle_count++;
            if (cycle_count > 1000000) {
                // Timeout — pipeline may be stuck
                uart_send_string("WARN: Pipeline timeout\r\n");
                break;
            }
        } while (!(irq_flags & IRQ_CLASS_DONE));

        // Clear IRQ flags
        USER_writeWord(irq_flags, SE_IRQ_FLAGS);

        // Read classification result
        result = USER_readWord(SE_CLASS_RESULT);
        class_id = GET_CLASS_ID(result);
        confidence = GET_CONFIDENCE(result);

        // Read status (check alarm)
        status = USER_readWord(SE_STATUS);

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

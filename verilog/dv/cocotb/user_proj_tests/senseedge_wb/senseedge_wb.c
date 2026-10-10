// SPDX-License-Identifier: Apache-2.0
// SenseEdge - Caravel co-simulation firmware (corrected RTL, branch fix/audit-defects)
//
// The management core is the only bus master the silicon has, so this is the test that
// matters: word-aligned Wishbone writes with byte selects, through the real Caravel.
//   1. load the 212 weights as 53 words and read every word back
//   2. configure divider and alarm, enable the chain
//   3. wait for the decision flag, read the 32 bins, 8 features and the class register
//   4. compare everything with the values the RTL produced for the same record
//      (verilog/dv/audit: tone at bin 8, amplitude 500, divider 4, diagonal weight image)
// Management GPIO: 1 = running, 0 = weights verified, 1 = PASS; on any failure the GPIO
// toggles forever and the cocotb side times out.

#include <firmware_apis.h>

#define SE_BASE         0x30000000u
#define SE_REG(off)     (*(volatile unsigned int *)(SE_BASE + (off)))
#define SE_CTRL         0x00
#define SE_STATUS       0x04
#define SE_CLASS        0x08
#define SE_ALARM_CFG    0x0C
#define SE_FFT_DATA     0x10
#define SE_FEAT_DATA    0x14
#define SE_IRQ_FLAGS    0x18
#define SE_CLK_DIV      0x1C
#define SE_WEIGHT(w)    (0x20 + 4 * (w))

static const unsigned short exp_bins[32] = {
    0, 0, 0, 0, 0, 0, 0, 0, 16010, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 10, 0, 0, 0, 0, 0, 0, 0 };
static const unsigned char exp_feat[8] = { 0, 62, 0, 0, 64, 62, 1, 62 };
#define EXP_CLASS 1
#define EXP_CONF  255

static void fail_forever(void)
{
    volatile int i;
    while (1) { ManagmentGpio_write(1); for (i = 0; i < 200; i++); ManagmentGpio_write(0); for (i = 0; i < 200; i++); }
}

void main(void)
{
    int i, w, errors = 0;
    unsigned int v;
    unsigned char wimg[212];

    ManagmentGpio_outputEnable();
    ManagmentGpio_write(0);
    enableHkSpi(0);

    GPIOs_configure(0, GPIO_MODE_USER_STD_INPUT_NOPULL);   // MISO
    GPIOs_configure(1, GPIO_MODE_USER_STD_OUTPUT);         // SCLK
    GPIOs_configure(2, GPIO_MODE_USER_STD_OUTPUT);         // CS_N
    GPIOs_configure(3, GPIO_MODE_USER_STD_OUTPUT);         // alarm
    GPIOs_configure(4, GPIO_MODE_USER_STD_OUTPUT);         // status LED
    GPIOs_configure(5, GPIO_MODE_USER_STD_OUTPUT);
    GPIOs_configure(6, GPIO_MODE_USER_STD_INPUT_NOPULL);
    GPIOs_loadConfigs();
    User_enableIF();

    ManagmentGpio_write(1);                                 // running

    // --- 1. weights: neuron k <- input k, class k <- hidden k ---
    for (i = 0; i < 212; i++) wimg[i] = 0;
    wimg[0] = 127; wimg[9] = 127; wimg[18] = 127; wimg[27] = 127;
    wimg[144] = 127; wimg[161] = 127; wimg[178] = 127; wimg[195] = 127;
    for (w = 0; w < 53; w++)
        SE_REG(SE_WEIGHT(w)) = (unsigned int)wimg[4*w] | ((unsigned int)wimg[4*w+1] << 8) |
                               ((unsigned int)wimg[4*w+2] << 16) | ((unsigned int)wimg[4*w+3] << 24);
    for (w = 0; w < 53; w++) {
        unsigned int e = (unsigned int)wimg[4*w] | ((unsigned int)wimg[4*w+1] << 8) |
                         ((unsigned int)wimg[4*w+2] << 16) | ((unsigned int)wimg[4*w+3] << 24);
        if (SE_REG(SE_WEIGHT(w)) != e) errors++;
    }
    if (SE_REG(0xF4) != 0) errors++;                        // above the window reads zero
    if (errors) fail_forever();
    ManagmentGpio_write(0);                                 // weights verified

    // --- 2. configure and enable ---
    SE_REG(SE_CLK_DIV)  = 4;
    SE_REG(SE_ALARM_CFG) = (3 << 8) | 100;
    SE_REG(SE_IRQ_FLAGS) = 0x3;
    SE_REG(SE_CTRL) = 1;

    // --- 3. wait for the decision flag ---
    for (i = 0; i < 400000; i++)
        if (SE_REG(SE_IRQ_FLAGS) & 0x1) break;
    if (!(SE_REG(SE_IRQ_FLAGS) & 0x1)) fail_forever();
    SE_REG(SE_CTRL) = 0;                                    // stop so that the read-back is of this window

    // --- 4. compare ---
    SE_REG(SE_FFT_DATA) = 0;
    for (i = 0; i < 32; i++) { v = SE_REG(SE_FFT_DATA) & 0xFFFF; if (v != exp_bins[i]) errors++; }
    SE_REG(SE_FEAT_DATA) = 0;
    for (i = 0; i < 8; i++)  { v = SE_REG(SE_FEAT_DATA) & 0xFF;   if (v != exp_feat[i]) errors++; }
    v = SE_REG(SE_CLASS);
    if ((v & 0x3) != EXP_CLASS || ((v >> 2) & 0xFF) != EXP_CONF) errors++;
    if (errors) fail_forever();

    ManagmentGpio_write(1);                                 // PASS
    while (1);
}

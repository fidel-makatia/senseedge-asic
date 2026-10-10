// SPDX-License-Identifier: Apache-2.0
// SenseEdge - Caravel co-simulation 2 (corrected RTL, branch fix/audit-defects)
// Several windows through the real management core: the alarm after N consecutive faults,
// its clearing on a healthy decision, and a register-map weight update that flips the class.
//
// Phases are signalled on the management GPIO and the test side switches the ADC record:
//   mgmt 1 : running                   (test serves the bin-8 record: band 1)
//   mgmt 0 : weights A verified        (test keeps bin 8)
//            windows: class 1 x4, alarm asserted after the 4th (N = 3), checked on STATUS
//   mgmt 1 : alarm verified            (test switches to the bin-2 record: band 0)
//            skip one mixed window; then class 0 x2, alarm cleared
//   mgmt 0 : clear verified; weights B loaded and verified   (test switches back to bin 8)
//            skip one mixed window; then class 2 x2 (B maps hidden 1 to class 2, hidden 0 to class 0)
//   mgmt 1 : PASS (steady).  Any failure: GPIO toggles forever.
#include <firmware_apis.h>

#define SE_BASE         0x30000000u
#define SE_REG(off)     (*(volatile unsigned int *)(SE_BASE + (off)))
#define SE_CTRL 0x00
#define SE_STATUS 0x04
#define SE_CLASS 0x08
#define SE_ALARM_CFG 0x0C
#define SE_IRQ_FLAGS 0x18
#define SE_CLK_DIV 0x1C
#define SE_WEIGHT(w) (0x20 + 4 * (w))

static unsigned char wimg[212];

static void fail_forever(void)
{
    volatile int i;
    while (1) { ManagmentGpio_write(1); for (i = 0; i < 200; i++); ManagmentGpio_write(0); for (i = 0; i < 200; i++); }
}

static int load_and_verify(void)
{
    int w, errors = 0;
    for (w = 0; w < 53; w++)
        SE_REG(SE_WEIGHT(w)) = (unsigned int)wimg[4*w] | ((unsigned int)wimg[4*w+1] << 8) |
                               ((unsigned int)wimg[4*w+2] << 16) | ((unsigned int)wimg[4*w+3] << 24);
    for (w = 0; w < 53; w++) {
        unsigned int e = (unsigned int)wimg[4*w] | ((unsigned int)wimg[4*w+1] << 8) |
                         ((unsigned int)wimg[4*w+2] << 16) | ((unsigned int)wimg[4*w+3] << 24);
        if (SE_REG(SE_WEIGHT(w)) != e) errors++;
    }
    return errors;
}

// wait for a decision, clear the flag, return the CLASS register
static unsigned int next_decision(void)
{
    int i;
    for (i = 0; i < 400000; i++)
        if (SE_REG(SE_IRQ_FLAGS) & 0x1) break;
    if (!(SE_REG(SE_IRQ_FLAGS) & 0x1)) fail_forever();
    SE_REG(SE_IRQ_FLAGS) = 0x1;
    return SE_REG(SE_CLASS);
}

static void expect(unsigned int v, unsigned int cls, unsigned int conf)
{
    if ((v & 0x3) != cls || ((v >> 2) & 0xFF) != conf) fail_forever();
}

void main(void)
{
    int i, k;
    unsigned int v;

    ManagmentGpio_outputEnable();
    ManagmentGpio_write(0);
    enableHkSpi(0);
    GPIOs_configure(0, GPIO_MODE_USER_STD_INPUT_NOPULL);
    GPIOs_configure(1, GPIO_MODE_USER_STD_OUTPUT);
    GPIOs_configure(2, GPIO_MODE_USER_STD_OUTPUT);
    GPIOs_configure(3, GPIO_MODE_USER_STD_OUTPUT);
    GPIOs_configure(4, GPIO_MODE_USER_STD_OUTPUT);
    GPIOs_configure(5, GPIO_MODE_USER_STD_OUTPUT);
    GPIOs_configure(6, GPIO_MODE_USER_STD_INPUT_NOPULL);
    GPIOs_loadConfigs();
    User_enableIF();
    ManagmentGpio_write(1);                                 // running

    // weights A: neuron k <- input k, class k <- hidden k
    for (i = 0; i < 212; i++) wimg[i] = 0;
    wimg[0] = 127; wimg[9] = 127; wimg[18] = 127; wimg[27] = 127;
    wimg[144] = 127; wimg[161] = 127; wimg[178] = 127; wimg[195] = 127;
    if (load_and_verify()) fail_forever();
    ManagmentGpio_write(0);                                 // A verified

    SE_REG(SE_CLK_DIV) = 4;
    SE_REG(SE_ALARM_CFG) = (3 << 8) | 100;                  // threshold 100, N = 3
    SE_REG(SE_IRQ_FLAGS) = 0x3;
    SE_REG(SE_CTRL) = 1;

    // four fault windows (bin 8 -> class 1); the alarm asserts on the fourth
    for (k = 0; k < 4; k++) {
        v = next_decision(); expect(v, 1, 255);
        v = SE_REG(SE_STATUS) & 0x10;
        if (k < 3 && v) fail_forever();                     // not yet
        if (k == 3 && !v) fail_forever();                   // now
    }
    ManagmentGpio_write(1);                                 // alarm verified -> test switches to bin 2

    next_decision();                                        // the window that straddles the switch
    for (k = 0; k < 2; k++) { v = next_decision(); expect(v, 0, 255); }
    if (SE_REG(SE_STATUS) & 0x10) fail_forever();           // alarm cleared by a confident healthy decision

    // weights B: hidden 0 stays class 0 (so the bin-2 record stays healthy and the alarm
    // stays clear while B is loaded), hidden 1 -> class 2, hidden 2 -> class 3, hidden 3 -> class 0.
    // The bin-8 record (hidden 1) therefore flips from class 1 under A to class 2 under B.
    for (i = 144; i < 212; i++) wimg[i] = 0;
    wimg[144 + 16 * 0 + 0] = 127;
    wimg[144 + 16 * 2 + 1] = 127;
    wimg[144 + 16 * 3 + 2] = 127;
    wimg[144 + 16 * 0 + 3] = 127;
    if (load_and_verify()) fail_forever();
    ManagmentGpio_write(0);                                 // B verified -> test switches to bin 8

    next_decision();                                        // straddling window
    for (k = 0; k < 2; k++) { v = next_decision(); expect(v, 2, 255); }

    SE_REG(SE_CTRL) = 0;
    ManagmentGpio_write(1);                                 // PASS
    while (1);
}

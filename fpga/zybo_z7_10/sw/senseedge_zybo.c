// SPDX-License-Identifier: Apache-2.0
// SenseEdge on the Zybo Z7-10: bare-metal program for the Zynq PS (Vitis, standalone BSP).
// The ARM core plays the Caravel management core: word-aligned AXI-Lite writes with byte
// strobes reach the chip's Wishbone registers through fpga/zybo_z7_10/rtl/axil_senseedge.v.
//
//   SW0 = 0  self-test: the on-chip MCP3201 emulator replays the bin-8 record of the audit,
//            the diagonal weight image is loaded as 53 words and read back, and the bins,
//            features and class are compared with the RTL values (PASS/FAIL on the UART);
//            then the chip runs continuously on the emulator and one CLASS line is printed
//            per window.
//   SW0 = 1  deployment: the ADC on Pmod JE is the source, the shipped weights of
//            firmware/nn_weights.h are loaded, and CLASS lines are printed as the Caravel
//            firmware prints them.
// Serial: PS UART1 through the USB-UART, 115200 8N1.
#include <stdint.h>
#include "xil_io.h"
#include "xil_printf.h"
#include "sleep.h"
#include "../../../firmware/nn_weights.h"      /* int8_t all_weights[212] */

#define SE_BASE     0x43C00000u                /* set by scripts/build.tcl */
#define WR(off, v)  Xil_Out32(SE_BASE + (off), (uint32_t)(v))
#define RD(off)     Xil_In32(SE_BASE + (off))

/* the chip's registers (wb_interface.v) */
#define SE_CTRL        0x00
#define SE_STATUS      0x04
#define SE_CLASS       0x08
#define SE_ALARM_CFG   0x0C
#define SE_FFT_DATA    0x10
#define SE_FEAT_DATA   0x14
#define SE_IRQ_FLAGS   0x18
#define SE_CLK_DIV     0x1C
#define SE_WEIGHT(w)   (0x20 + 4 * (w))
/* the wrapper's registers */
#define EMU_CTRL       0x1000
#define EMU_LEN        0x1004
#define WRAP_ID        0x1008
#define WRAP_SW        0x100C
#define EMU_CODES      0x2000

static const uint16_t rec8[64] = { 2048, 2402, 2548, 2402, 2048, 1694, 1548, 1694, 2048, 2402, 2548, 2402, 2048, 1694, 1548, 1694, 2048, 2402, 2548, 2402, 2048, 1694, 1548, 1694, 2048, 2402, 2548, 2402, 2048, 1694, 1548, 1694, 2048, 2402, 2548, 2402, 2048, 1694, 1548, 1694, 2048, 2402, 2548, 2402, 2048, 1694, 1548, 1694, 2048, 2402, 2548, 2402, 2048, 1694, 1548, 1694, 2048, 2402, 2548, 2402, 2048, 1694, 1548, 1694 };
static const uint16_t exp_bins[32] = { 0,0,0,0,0,0,0,0,16010,0,0,0,0,0,0,0, 0,0,0,0,0,0,0,0,10,0,0,0,0,0,0,0 };
static const uint8_t  exp_feat[8]  = { 0, 62, 0, 0, 64, 62, 1, 62 };
static const char *class_names[4] = { "HEALTHY", "BEARING_WEAR", "IMBALANCE", "MISALIGNMENT" };

static uint32_t pack(const int8_t *w, int i)
{
    return ((uint32_t)(uint8_t)w[i]) | ((uint32_t)(uint8_t)w[i+1] << 8) |
           ((uint32_t)(uint8_t)w[i+2] << 16) | ((uint32_t)(uint8_t)w[i+3] << 24);
}

static int load_weights(const int8_t *w)
{
    int i, errors = 0;
    for (i = 0; i < 53; i++) WR(SE_WEIGHT(i), pack(w, 4 * i));
    for (i = 0; i < 53; i++) if (RD(SE_WEIGHT(i)) != pack(w, 4 * i)) errors++;
    return errors;
}

static int wait_decision(void)
{
    int i;
    for (i = 0; i < 20000000; i++) {
        if (RD(SE_IRQ_FLAGS) & 1u) { WR(SE_IRQ_FLAGS, 1u); return 1; }
    }
    return 0;
}

static void print_class_line(void)
{
    uint32_t v = RD(SE_CLASS), st = RD(SE_STATUS);
    xil_printf("CLASS:%s CONF:%u ALARM:%u\r\n", class_names[v & 3], (unsigned)((v >> 2) & 0xFF), (unsigned)((st >> 4) & 1));
}

static int self_test(void)
{
    int i, errors = 0;
    int8_t diag[212];
    uint32_t v;
    for (i = 0; i < 212; i++) diag[i] = 0;
    diag[0] = diag[9] = diag[18] = diag[27] = 127;        /* neuron k <- input k   */
    diag[144] = diag[161] = diag[178] = diag[195] = 127;  /* class k  <- hidden k  */

    for (i = 0; i < 64; i++) WR(EMU_CODES + 4 * i, rec8[i]);
    WR(EMU_LEN, 64);
    WR(EMU_CTRL, 1);                                      /* emulator feeds the capture unit */
    i = load_weights(diag);
    xil_printf("weights: 53 words written, %d read-back errors\r\n", i);
    errors += i;
    WR(SE_CLK_DIV, 4);
    WR(SE_ALARM_CFG, (3u << 8) | 100u);
    WR(SE_IRQ_FLAGS, 3);
    WR(SE_CTRL, 1);
    if (!wait_decision()) { xil_printf("FAIL: no decision\r\n"); return errors + 1; }
    WR(SE_CTRL, 0);                                       /* freeze this window for read-back */
    WR(SE_FFT_DATA, 0);
    for (i = 0; i < 32; i++) { v = RD(SE_FFT_DATA) & 0xFFFF; if (v != exp_bins[i]) { xil_printf("  bin %d = %u (expected %u)\r\n", i, (unsigned)v, exp_bins[i]); errors++; } }
    WR(SE_FEAT_DATA, 0);
    for (i = 0; i < 8; i++) { v = RD(SE_FEAT_DATA) & 0xFF; if (v != exp_feat[i]) { xil_printf("  feature %d = %u (expected %u)\r\n", i, (unsigned)v, exp_feat[i]); errors++; } }
    v = RD(SE_CLASS);
    if ((v & 3) != 1 || ((v >> 2) & 0xFF) != 255) { xil_printf("  class %u conf %u (expected 1, 255)\r\n", (unsigned)(v & 3), (unsigned)((v >> 2) & 0xFF)); errors++; }
    xil_printf("self-test: 32 bins, 8 features, class: %s\r\n", errors ? "FAIL" : "PASS, identical to the RTL audit values");
    return errors;
}

int main(void)
{
    uint32_t sw;
    xil_printf("\r\nSenseEdge on Zybo Z7-10, wrapper id 0x%08x\r\n", (unsigned)RD(WRAP_ID));
    sw = RD(WRAP_SW) & 3u;
    xil_printf("SW0=%u: %s\r\n", (unsigned)(sw & 1), (sw & 1) ? "external ADC on Pmod JE" : "on-chip emulator self-test");

    if (!(sw & 1)) {
        int e = self_test();
        xil_printf(e ? "*** SELF-TEST FAILED (%d) ***\r\n" : "*** SELF-TEST PASSED ***\r\n", e);
        WR(SE_IRQ_FLAGS, 3);
        WR(SE_CTRL, 1);                                   /* keep running on the emulator */
    } else {
        int e;
        WR(EMU_CTRL, 0);
        e = load_weights(all_weights);
        xil_printf("WEIGHTS verified, word errors: %d\r\n", e);
        WR(SE_CLK_DIV, 249);                              /* 20 MHz / (35 * 250) = 2.29 kS/s, as the chip's reset value */
        WR(SE_ALARM_CFG, (3u << 8) | 150u);
        WR(SE_IRQ_FLAGS, 3);
        WR(SE_CTRL, 1);
        xil_printf("Monitoring vibration...\r\n");
    }
    while (1) {
        if (wait_decision()) print_class_line();
        else xil_printf("WARN: pipeline timeout\r\n");
    }
    return 0;
}

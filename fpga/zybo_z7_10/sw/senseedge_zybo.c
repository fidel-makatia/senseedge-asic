// SPDX-License-Identifier: Apache-2.0
// SenseEdge on the Zybo Z7-10: bare-metal program for the Zynq PS (Vitis, standalone BSP).
// The ARM core plays the Caravel management core: word-aligned AXI-Lite writes with byte
// strobes reach the chip's Wishbone registers through fpga/zybo_z7_10/rtl/axil_senseedge.v.
//
// Text mode (power-up), one CLASS line per decision on the UART:
//   SW0 = 0  self-test: the on-chip MCP3201 emulator replays the bin-8 record of the audit,
//            the diagonal weight image is loaded as 53 words and read back, and the bins,
//            features and class are compared with the RTL values (PASS/FAIL on the UART);
//            then the chip runs continuously on the emulator.
//   SW0 = 1  deployment: the ADC on Pmod JE is the source and the shipped weights of
//            firmware/nn_weights.h are loaded.
// Host mode, entered by the first frame byte 0xA5 from host/se_link.py (dataset replay, the
// register-map update test, the divider check):
//   host -> board   A5 cmd len_lo len_hi payload[len] crc      crc = XOR of cmd, len, payload
//   board -> host   5A status len_lo len_hi payload[len] crc
//   'I' identify            -> id u32, protocol u32, switches u32, host_mode u8
//   'W' weights[212]        -> status = read-back errors; payload = the 53 words read back
//   'C' div u16, alarm u16  -> divider and alarm config, kept and re-applied after a reset
//   'X'                     -> soft reset of chip + emulator, weights and config reloaded
//   'R' n u8, n*64 codes u16-> codes into the emulator RAM, EMU_LEN = 64 n, emulator selected
//   'G' n u8, flags u8      -> reset + reload, run n windows, per window: class, conf, alarm,
//                              [8 features if flags&1], [32 bins u16 if flags&2]; payload
//                              starts with the number of windows completed; status 1 = timeout
//   'D' off u16             -> one register, u32        'P' off u16, val u32 -> write it
//   'Q'                     -> back to text mode (chip re-enabled on the selected source)
// Serial: PS UART1 through the USB-UART, 115200 8N1 unless SE_BAUD is defined at compile time
// (921600 cuts a replay's transfer time by 8; set the host script's --baud to match).
#include <stdint.h>
#include <string.h>
#include "xil_io.h"
#include "xil_printf.h"
#include "xparameters.h"
#include "xuartps_hw.h"
#include "sleep.h"
#include "../../../firmware/nn_weights.h"      /* int8_t all_weights[212] */

#ifndef SE_UART_BASE
#define SE_UART_BASE STDIN_BASEADDRESS
#endif
#ifdef SE_BAUD
#include "xuartps.h"
#if defined(SDT)
#define SE_UART_ID SE_UART_BASE            /* system-device-tree BSPs (Vitis 2023.2+): LookupConfig takes the base address */
#elif defined(XPAR_PS7_UART_1_DEVICE_ID)
#define SE_UART_ID XPAR_PS7_UART_1_DEVICE_ID
#else
#define SE_UART_ID XPAR_XUARTPS_0_DEVICE_ID
#endif
#endif

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

#define PROTOCOL_VERSION 1u
#define MAX_PAYLOAD      (1 + 64 * 128)         /* 'R' with 64 windows */
#define MAX_RESP         (1 + 64 * (3 + 8 + 64))
#define RX_SPINS         20000000u

static const uint16_t rec8[64] = { 2048, 2402, 2548, 2402, 2048, 1694, 1548, 1694, 2048, 2402, 2548, 2402, 2048, 1694, 1548, 1694, 2048, 2402, 2548, 2402, 2048, 1694, 1548, 1694, 2048, 2402, 2548, 2402, 2048, 1694, 1548, 1694, 2048, 2402, 2548, 2402, 2048, 1694, 1548, 1694, 2048, 2402, 2548, 2402, 2048, 1694, 1548, 1694, 2048, 2402, 2548, 2402, 2048, 1694, 1548, 1694, 2048, 2402, 2548, 2402, 2048, 1694, 1548, 1694 };
static const uint16_t exp_bins[32] = { 0,0,0,0,0,0,0,0,16010,0,0,0,0,0,0,0, 0,0,0,0,0,0,0,0,10,0,0,0,0,0,0,0 };
static const uint8_t  exp_feat[8]  = { 0, 62, 0, 0, 64, 62, 1, 62 };
static const char *class_names[4] = { "HEALTHY", "BEARING_WEAR", "IMBALANCE", "MISALIGNMENT" };

/* state kept across soft resets */
static uint8_t  cur_w[212];
static uint16_t cur_div = 249;
static uint16_t cur_alarm = (3u << 8) | 150u;
static uint8_t  emu_sel = 0;
static int      host_mode = 0;
static uint8_t  pay[MAX_PAYLOAD];
static uint8_t  resp[MAX_RESP];

/* ------------------------------------------------------------------ chip helpers */
static uint32_t pack(const uint8_t *w, int i)
{
    return ((uint32_t)w[i]) | ((uint32_t)w[i+1] << 8) | ((uint32_t)w[i+2] << 16) | ((uint32_t)w[i+3] << 24);
}

static int load_weights(const uint8_t *w)
{
    int i, errors = 0;
    for (i = 0; i < 53; i++) WR(SE_WEIGHT(i), pack(w, 4 * i));
    for (i = 0; i < 53; i++) if (RD(SE_WEIGHT(i)) != pack(w, 4 * i)) errors++;
    return errors;
}

static void apply_config(void)
{
    WR(SE_CLK_DIV, cur_div);
    WR(SE_ALARM_CFG, cur_alarm);
}

/* Reset the chip and the emulator (EMU_CTRL bit 1), then reload weights and configuration.
   Every replay block starts here, so the emulator plays code 0 into an empty sample buffer. */
static int chip_reset_reload(void)
{
    int i;
    WR(EMU_CTRL, (uint32_t)emu_sel | 2u);
    for (i = 0; i < 1000 && (RD(EMU_CTRL) & 2u); i++) ;
    i = load_weights(cur_w);
    apply_config();
    WR(SE_IRQ_FLAGS, 3);
    return i;
}

static int wait_decision(void)
{
    uint32_t i;
    for (i = 0; i < RX_SPINS; i++) {
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
    uint32_t v;
    memset(cur_w, 0, sizeof cur_w);
    cur_w[0] = cur_w[9] = cur_w[18] = cur_w[27] = 127;        /* neuron k <- input k   */
    cur_w[144] = cur_w[161] = cur_w[178] = cur_w[195] = 127;  /* class k  <- hidden k  */

    for (i = 0; i < 64; i++) WR(EMU_CODES + 4 * i, rec8[i]);
    WR(EMU_LEN, 64);
    emu_sel = 1;
    WR(EMU_CTRL, 1);                                      /* emulator feeds the capture unit */
    i = load_weights(cur_w);
    xil_printf("weights: 53 words written, %d read-back errors\r\n", i);
    errors += i;
    cur_div = 4; cur_alarm = (3u << 8) | 100u;
    apply_config();
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

/* ------------------------------------------------------------------ UART and frames */
static int uart_rx_ready(void) { return XUartPs_IsReceiveData(SE_UART_BASE) ? 1 : 0; }
static uint8_t uart_rx(void) { return XUartPs_RecvByte(SE_UART_BASE); }
static void uart_tx(uint8_t b) { XUartPs_SendByte(SE_UART_BASE, b); }

static int uart_rx_timeout(uint8_t *b)
{
    uint32_t i;
    for (i = 0; i < RX_SPINS; i++) if (uart_rx_ready()) { *b = uart_rx(); return 1; }
    return 0;
}

static void uart_set_baud(void)
{
#ifdef SE_BAUD
    static XUartPs u;
    XUartPs_Config *c = XUartPs_LookupConfig(SE_UART_ID);
    if (c && XUartPs_CfgInitialize(&u, c, c->BaseAddress) == XST_SUCCESS) XUartPs_SetBaudRate(&u, SE_BAUD);
#endif
}

static void send_frame(uint8_t status, const uint8_t *p, uint32_t len)
{
    uint32_t i; uint8_t crc = status ^ (uint8_t)len ^ (uint8_t)(len >> 8);
    uart_tx(0x5A); uart_tx(status); uart_tx((uint8_t)len); uart_tx((uint8_t)(len >> 8));
    for (i = 0; i < len; i++) { uart_tx(p[i]); crc ^= p[i]; }
    uart_tx(crc);
}

static void put32(uint8_t *p, uint32_t v) { p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8); p[2] = (uint8_t)(v >> 16); p[3] = (uint8_t)(v >> 24); }
static uint32_t get16(const uint8_t *p) { return (uint32_t)p[0] | ((uint32_t)p[1] << 8); }
static uint32_t get32(const uint8_t *p) { return get16(p) | (get16(p + 2) << 16); }

/* Run n windows from a clean reset; records into resp[1..]; returns status, resp[0] = done. */
static uint8_t run_windows(uint32_t n, uint32_t flags, uint32_t *out_len)
{
    uint32_t w, i, pos = 1, v;
    uint8_t status = 0;
    if (n > 64) n = 64;
    if (chip_reset_reload()) status = 2;
    WR(SE_CTRL, 1);
    for (w = 0; w < n; w++) {
        if (!wait_decision()) { status = 1; break; }
        v = RD(SE_CLASS);
        resp[pos++] = (uint8_t)(v & 3);
        resp[pos++] = (uint8_t)((v >> 2) & 0xFF);
        resp[pos++] = (uint8_t)((RD(SE_STATUS) >> 4) & 1);
        if (flags & 1) {
            WR(SE_FEAT_DATA, 0);
            for (i = 0; i < 8; i++) resp[pos++] = (uint8_t)(RD(SE_FEAT_DATA) & 0xFF);
        }
        if (flags & 2) {
            WR(SE_FFT_DATA, 0);
            for (i = 0; i < 32; i++) { v = RD(SE_FFT_DATA) & 0xFFFF; resp[pos++] = (uint8_t)v; resp[pos++] = (uint8_t)(v >> 8); }
        }
    }
    WR(SE_CTRL, 0);
    resp[0] = (uint8_t)w;
    *out_len = pos;
    return status;
}

/* One frame after its 0xA5 start byte; replies with one frame. */
static void handle_frame(void)
{
    uint8_t cmd, b, crc = 0, rcrc;
    uint32_t len, i, n, rlen = 0;
    uint8_t status = 0;
    if (!uart_rx_timeout(&cmd)) return;
    if (!uart_rx_timeout(&b)) return; len = b;
    if (!uart_rx_timeout(&b)) return; len |= (uint32_t)b << 8;
    crc = cmd ^ (uint8_t)len ^ (uint8_t)(len >> 8);
    if (len > MAX_PAYLOAD) { for (i = 0; i < len; i++) if (!uart_rx_timeout(&b)) break; uart_rx_timeout(&b); send_frame(0xFE, 0, 0); return; }
    for (i = 0; i < len; i++) { if (!uart_rx_timeout(&pay[i])) return; crc ^= pay[i]; }
    if (!uart_rx_timeout(&rcrc)) return;
    if (rcrc != crc) { send_frame(0xFD, 0, 0); return; }
    host_mode = 1;
    switch (cmd) {
    case 'I':
        put32(resp, RD(WRAP_ID)); put32(resp + 4, PROTOCOL_VERSION); put32(resp + 8, RD(WRAP_SW) & 3u);
        resp[12] = (uint8_t)host_mode; rlen = 13; break;
    case 'W':
        if (len != 212) { status = 0xFC; break; }
        memcpy(cur_w, pay, 212);
        status = (uint8_t)load_weights(cur_w);
        for (i = 0; i < 53; i++) put32(resp + 4 * i, RD(SE_WEIGHT(i)));
        rlen = 212; break;
    case 'C':
        if (len != 4) { status = 0xFC; break; }
        cur_div = (uint16_t)get16(pay); cur_alarm = (uint16_t)get16(pay + 2);
        apply_config(); break;
    case 'X':
        status = (uint8_t)chip_reset_reload(); break;
    case 'R':
        n = len ? pay[0] : 0;
        if (n < 1 || n > 64 || len != 1 + 128 * n) { status = 0xFC; break; }
        for (i = 0; i < 64 * n; i++) WR(EMU_CODES + 4 * i, get16(pay + 1 + 2 * i) & 0xFFF);
        WR(EMU_LEN, 64 * n);
        emu_sel = 1; WR(EMU_CTRL, 1);
        resp[0] = (uint8_t)n; rlen = 1; break;
    case 'G':
        if (len != 2) { status = 0xFC; break; }
        status = run_windows(pay[0], pay[1], &rlen); break;
    case 'D':
        if (len != 2) { status = 0xFC; break; }
        put32(resp, RD(get16(pay))); rlen = 4; break;
    case 'P':
        if (len != 6) { status = 0xFC; break; }
        WR(get16(pay), get32(pay + 2)); break;
    case 'Q':
        host_mode = 0; WR(SE_IRQ_FLAGS, 3); WR(SE_CTRL, 1); break;
    default:
        status = 0xFF; break;
    }
    send_frame(status, resp, rlen);
}

/* ------------------------------------------------------------------ main */
int main(void)
{
    uint32_t sw;
    uart_set_baud();
    xil_printf("\r\nSenseEdge on Zybo Z7-10, wrapper id 0x%08x, protocol %u\r\n", (unsigned)RD(WRAP_ID), (unsigned)PROTOCOL_VERSION);
    sw = RD(WRAP_SW) & 3u;
    xil_printf("SW0=%u: %s\r\n", (unsigned)(sw & 1), (sw & 1) ? "external ADC on Pmod JE" : "on-chip emulator self-test");

    if (!(sw & 1)) {
        int e = self_test();
        xil_printf(e ? "*** SELF-TEST FAILED (%d) ***\r\n" : "*** SELF-TEST PASSED ***\r\n", e);
        WR(SE_IRQ_FLAGS, 3);
        WR(SE_CTRL, 1);                                   /* keep running on the emulator */
    } else {
        int e;
        emu_sel = 0;
        WR(EMU_CTRL, 0);
        memcpy(cur_w, all_weights, 212);
        e = load_weights(cur_w);
        xil_printf("WEIGHTS verified, word errors: %d\r\n", e);
        cur_div = 249;                                    /* 20 MHz / (35 * 250) = 2.29 kS/s, the chip's reset value */
        cur_alarm = (3u << 8) | 150u;
        apply_config();
        WR(SE_IRQ_FLAGS, 3);
        WR(SE_CTRL, 1);
        xil_printf("Monitoring vibration...\r\n");
    }
    while (1) {
        if (uart_rx_ready()) {
            if (uart_rx() == 0xA5) handle_frame();
        } else if (!host_mode) {
            if (RD(SE_IRQ_FLAGS) & 1u) { WR(SE_IRQ_FLAGS, 1u); print_class_line(); }
        }
    }
    return 0;
}

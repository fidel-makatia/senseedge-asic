/* Host-side mock of the Zybo wrapper + SenseEdge registers, for testing the firmware's UART
   protocol against host/se_link.py without a board.  The UART is stdin/stdout (binary); text
   output (xil_printf) goes to stderr.  The "chip" produces deterministic pseudo-results so the
   host test can check every byte that crosses the link:
     after reset, decision k (k = 0, 1, ...) reports class = k & 3, confidence = 100 + (k & 127),
     alarm = (k >> 2) & 1, feature i = ram[(64 k + i) % 4096] & 0xFF,
     bin i = (ram[(64 k + i) % 4096] ^ i) & 0xFFFF;  a decision is pending 3 polls after enable. */
#include <poll.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include "xil_io.h"
#include "xuartps.h"

#define BASE 0x43C00000u
static uint32_t ctrl, flags, divr = 249, alarm_cfg = (3u << 8) | 128u, wregs[53], emu_ctrl, emu_len = 64;
static uint32_t ram[4096], fft_ptr, feat_ptr, polls, decided = 0, have_decision = 0;

static void chip_reset(void) { ctrl = 0; flags = 0; divr = 249; alarm_cfg = (3u << 8) | 128u; memset(wregs, 0, sizeof wregs); decided = 0; have_decision = 0; polls = 0; }

void Xil_Out32(uint32_t a, uint32_t v)
{
    uint32_t off = a - BASE;
    if (off == 0x00) { ctrl = v & 1; polls = 0; }
    else if (off == 0x0C) alarm_cfg = v & 0xFFF;
    else if (off == 0x10) fft_ptr = 0;
    else if (off == 0x14) feat_ptr = 0;
    else if (off == 0x18) flags &= ~v;
    else if (off == 0x1C) divr = v & 0xFFFF;
    else if (off >= 0x20 && off <= 0xF0) wregs[(off - 0x20) / 4] = v;
    else if (off == 0x1000) { emu_ctrl = v & 1; if (v & 2) chip_reset(); }
    else if (off == 0x1004) emu_len = v & 0x1FFF;
    else if (off >= 0x2000 && off < 0x6000) ram[(off - 0x2000) / 4] = v & 0xFFF;
}

uint32_t Xil_In32(uint32_t a)
{
    uint32_t off = a - BASE, k = have_decision ? decided - 1 : 0, v;
    if (off == 0x00) return ctrl;
    if (off == 0x04) return ctrl | (((k >> 2) & 1) << 4);
    if (off == 0x08) return (k & 3) | ((100 + (k & 127)) << 2);
    if (off == 0x0C) return alarm_cfg;
    if (off == 0x10) { v = (ram[(64 * k + fft_ptr) % 4096] ^ fft_ptr) & 0xFFFF; fft_ptr = (fft_ptr + 1) & 31; return v; }
    if (off == 0x14) { v = ram[(64 * k + feat_ptr) % 4096] & 0xFF; feat_ptr = (feat_ptr + 1) & 7; return v; }
    if (off == 0x18) {
        if (ctrl && !(flags & 1)) { if (++polls >= 3) { polls = 0; flags |= 1; decided++; have_decision = 1; } else usleep(100); }
        return flags;
    }
    if (off == 0x1C) return divr;
    if (off >= 0x20 && off <= 0xF0) return wregs[(off - 0x20) / 4];
    if (off == 0x1000) return emu_ctrl;
    if (off == 0x1004) return emu_len;
    if (off == 0x1008) return 0x5E5E0002u;
    if (off == 0x100C) { const char *s = getenv("MOCK_SW"); return s ? (uint32_t)atoi(s) : 0; }
    if (off >= 0x2000 && off < 0x6000) return ram[(off - 0x2000) / 4];
    return 0;
}

void xil_printf(const char *fmt, ...) { va_list ap; va_start(ap, fmt); vfprintf(stderr, fmt, ap); va_end(ap); fflush(stderr); }

int XUartPs_IsReceiveData(uint32_t base) { struct pollfd p = { 0, POLLIN, 0 }; (void)base; if (poll(&p, 1, 0) > 0) return 1; usleep(200); return 0; }
uint8_t XUartPs_RecvByte(uint32_t base) { uint8_t b; (void)base; if (read(0, &b, 1) != 1) exit(0); return b; }
void XUartPs_SendByte(uint32_t base, uint8_t b) { (void)base; if (write(1, &b, 1) != 1) exit(1); }
static XUartPs_Config cfg = { 0xE0001000u };
XUartPs_Config *XUartPs_LookupConfig(unsigned id) { (void)id; return &cfg; }
int XUartPs_CfgInitialize(XUartPs *u, XUartPs_Config *c, uint32_t base) { (void)u; (void)c; (void)base; return 0; }
int XUartPs_SetBaudRate(XUartPs *u, uint32_t baud) { (void)u; fprintf(stderr, "[mock] baud %u\n", (unsigned)baud); return 0; }

#ifndef XUARTPS_H
#define XUARTPS_H
#include <stdint.h>
#include "xuartps_hw.h"
#define XST_SUCCESS 0
typedef struct { uint32_t BaseAddress; } XUartPs_Config;
typedef struct { int dummy; } XUartPs;
XUartPs_Config *XUartPs_LookupConfig(unsigned id);
int XUartPs_CfgInitialize(XUartPs *u, XUartPs_Config *c, uint32_t base);
int XUartPs_SetBaudRate(XUartPs *u, uint32_t baud);
#endif

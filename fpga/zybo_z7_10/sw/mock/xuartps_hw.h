#ifndef XUARTPS_HW_H
#define XUARTPS_HW_H
#include <stdint.h>
int XUartPs_IsReceiveData(uint32_t base);
uint8_t XUartPs_RecvByte(uint32_t base);
void XUartPs_SendByte(uint32_t base, uint8_t b);
#endif

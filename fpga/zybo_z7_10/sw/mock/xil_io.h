/* mock BSP header: registers go to mock_chip.c */
#ifndef XIL_IO_H
#define XIL_IO_H
#include <stdint.h>
typedef uint8_t u8; typedef uint16_t u16; typedef uint32_t u32;
uint32_t Xil_In32(uint32_t addr);
void Xil_Out32(uint32_t addr, uint32_t value);
#endif

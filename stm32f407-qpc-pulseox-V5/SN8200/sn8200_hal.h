#ifndef __SN8200_HAL_H
#define __SN8200_HAL_H
#include <stdbool.h>
#include "stm32f4xx.h"

void SN8200_HAL_Init(uint32_t baudrate);
void SN8200_HAL_SendData(unsigned char *buf, int len);
bool SN8200_HAL_RxBufferEmpty(void);
uint8_t SN8200_HAL_ReadByte(void);
#endif

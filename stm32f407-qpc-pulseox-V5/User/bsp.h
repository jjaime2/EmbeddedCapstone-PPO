#ifndef __BSP_H__
#define __BSP_H__

#include <stdint.h>
#include "qpc.h"
/* system clock tick [Hz] */
#define BSP_TICKS_PER_SEC 1000U


void BSP_ledBlueToggle(void);
void BSP_ledGreenToggle(void);
void BSP_ledRedToggle(void);
void BSP_ledOrangeToggle(void);

void TimingDelay_Decrement(void);
void mdelay(uint32_t ms);

extern QXSemaphore INT_sema;
extern QXSemaphore CALC_sema;

#endif // __BSP_H__

#include "stm32f4xx.h"
#include "pportos.h"

static uint32_t volatile l_tickCtr;
extern uint32_t SystemCoreClock;

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
void SysTick_Handler(void)
{
  ++l_tickCtr;
	
	__disable_irq();
	OS_sched();
	__enable_irq();
}

void SysTick_Configuration(void)
{
		SystemCoreClockUpdate();
    SysTick_Config(SystemCoreClock / 1000);
		NVIC_SetPriority(SysTick_IRQn, 0U);
	
		__enable_irq();
}

uint32_t BSP_tickCtr(void) {
    uint32_t tickCtr;

    __disable_irq();
    tickCtr = l_tickCtr;
    __enable_irq();

    return tickCtr;
}

void mdelay(uint32_t ticks)
{
    uint32_t start = BSP_tickCtr();
    while ((BSP_tickCtr() - start) < ticks) {
    }
}
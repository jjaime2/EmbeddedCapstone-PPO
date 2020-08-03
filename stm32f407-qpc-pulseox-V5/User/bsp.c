#include "stm32f4xx.h"
#include "qpc.h"
#include "bsp.h"

#define GPIO_LED									GPIOD
#define GPIO_Pin_LED_GREEN				GPIO_Pin_12
#define GPIO_Pin_LED_ORANGE				GPIO_Pin_13
#define GPIO_Pin_LED_RED					GPIO_Pin_14
#define GPIO_Pin_LED_BLUE					GPIO_Pin_15

static uint32_t TimingDelay;
extern uint32_t SystemCoreClock;

void mdelay(uint32_t ms)
{
    TimingDelay = ms;

    while(TimingDelay != 0) TimingDelay--;
}

void TimingDelay_Decrement(void)
{
    if (TimingDelay != 0x00) {
        TimingDelay--;
    }
}

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
void SysTick_Handler(void)
{
	QXK_ISR_ENTRY();
	
  QF_TICK_X(0U, (void*)0);
	//TimingDelay_Decrement();
	
	QXK_ISR_EXIT();
}

void BSP_ledBlueToggle(void) {
	QF_CRIT_STAT_TYPE istat;
	
	QF_CRIT_ENTRY(istat);
	GPIO_ToggleBits(GPIO_LED, GPIO_Pin_LED_BLUE);
	QF_CRIT_EXIT(istat);
}

void BSP_ledGreenToggle(void) {
	QF_CRIT_STAT_TYPE istat;
	
	QF_CRIT_ENTRY(istat);
	GPIO_ToggleBits(GPIO_LED, GPIO_Pin_LED_GREEN);
	QF_CRIT_EXIT(istat);
}

void BSP_ledRedToggle(void) {
	QF_CRIT_STAT_TYPE istat;
	
	QF_CRIT_ENTRY(istat);
	GPIO_ToggleBits(GPIO_LED, GPIO_Pin_LED_RED);
	QF_CRIT_EXIT(istat);
}

void BSP_ledOrangeToggle(void) {
	QF_CRIT_STAT_TYPE istat;
	
	QF_CRIT_ENTRY(istat);
	GPIO_ToggleBits(GPIO_LED, GPIO_Pin_LED_ORANGE);
	QF_CRIT_EXIT(istat);
}

void QF_onStartup() {
	SystemCoreClockUpdate();
	SysTick_Config(SystemCoreClock / BSP_TICKS_PER_SEC);
}

void QF_onCleanup(void) {
	
}

void QXK_onIdle(void) {
	//__WFI();
}

void Q_onAssert(char const *module, int loc) {
	(void)module;
	(void)loc;
	NVIC_SystemReset();
}

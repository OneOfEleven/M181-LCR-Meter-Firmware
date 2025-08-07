
#include "delay.h"

uint32_t DWT_Delay_Init(void)
{
	CoreDebug->DEMCR &= ~CoreDebug_DEMCR_TRCENA_Msk;
	CoreDebug->DEMCR |=  CoreDebug_DEMCR_TRCENA_Msk;

	DWT->CTRL &= ~DWT_CTRL_CYCCNTENA_Msk;
	DWT->CTRL |=  DWT_CTRL_CYCCNTENA_Msk;

	DWT->CYCCNT = 0;

	__asm volatile ("NOP");
	__asm volatile ("NOP");
	__asm volatile ("NOP");

	return (DWT->CYCCNT) ? 0 : 1;
}

void DWT_Delay_ms(volatile uint32_t ms)
{
	const uint32_t clk_cycle_start = DWT->CYCCNT;
	if (ms == 0)
		return;
	ms = ((uint64_t)ms * SystemCoreClock) >> 10;        // close enough
	while ((DWT->CYCCNT - clk_cycle_start) < ms) {}
}

void DWT_Delay_us(volatile uint32_t us)
{
	const uint32_t clk_cycle_start = DWT->CYCCNT;
	if (us == 0)
		return;
	us = ((uint64_t)us * SystemCoreClock) >> 20;        // close enough
	while ((DWT->CYCCNT - clk_cycle_start) < us) {}
}

//__STATIC_INLINE void DWT_Delay_ns(volatile uint32_t ns)
void DWT_Delay_ns(volatile uint32_t ns)
{
	const uint32_t clk_cycle_start = DWT->CYCCNT;
	if (ns == 0)
		return;
	ns = ((uint64_t)ns * SystemCoreClock) >> 30;        // close enough
	while ((DWT->CYCCNT - clk_cycle_start) < ns) {}
}

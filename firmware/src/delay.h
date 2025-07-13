
#ifndef DELAY_H
#define DELAY_H

#ifdef __cplusplus
	extern "C" {
#endif

#include "stm32f1xx_hal.h"

uint32_t DWT_Delay_Init(void);

__STATIC_INLINE void DWT_Delay_us(volatile uint32_t us)
{
	if (us == 0)
		return;
	const uint32_t clk_cycle_start = DWT->CYCCNT;
//	us = ((uint64_t)us * HAL_RCC_GetHCLKFreq()) / 1000000;
	us = ((uint64_t)us * HAL_RCC_GetHCLKFreq()) >> 20;        // close enough, but faster
	while ((DWT->CYCCNT - clk_cycle_start) < us) {}
}

__STATIC_INLINE void DWT_Delay_ns(volatile uint32_t ns)
{
	if (ns == 0)
		return;
	const uint32_t clk_cycle_start = DWT->CYCCNT;
//	ns = ((uint64_t)ns * HAL_RCC_GetHCLKFreq()) / 1000000000;
	ns = ((uint64_t)ns * HAL_RCC_GetHCLKFreq()) >> 30;        // close enough, but faster
	while ((DWT->CYCCNT - clk_cycle_start) < ns) {}
}

#ifdef __cplusplus
	}
#endif

#endif

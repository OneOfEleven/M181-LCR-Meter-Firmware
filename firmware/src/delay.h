
#ifndef DELAY_H
#define DELAY_H

#ifdef __cplusplus
	extern "C" {
#endif

#include <stdint.h>

#include "stm32f1xx_hal.h"

uint32_t DWT_Delay_Init(void);
void DWT_Delay_ms(volatile uint32_t ms);
void DWT_Delay_us(volatile uint32_t us);
void DWT_Delay_ns(volatile uint32_t ns);

/*
__STATIC_INLINE void DWT_Delay_ns(volatile uint32_t ns)
{
	const uint32_t clk_cycle_start = DWT->CYCCNT;
	if (ns == 0)
		return;
	ns = ((uint64_t)ns * SystemCoreClock) >> 30;        // close enough
	while ((DWT->CYCCNT - clk_cycle_start) < ns) {}
}
*/

#ifdef __cplusplus
	}
#endif

#endif

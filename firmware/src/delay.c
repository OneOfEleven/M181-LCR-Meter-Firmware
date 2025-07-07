
#include "delay.h"

uint32_t DWT_Delay_Init(void)
{
	CoreDebug->DEMCR &= ~CoreDebug_DEMCR_TRCENA_Msk; // ~0x01000000;
	CoreDebug->DEMCR |=  CoreDebug_DEMCR_TRCENA_Msk; // 0x01000000;
	
	DWT->CTRL &= ~DWT_CTRL_CYCCNTENA_Msk; //~0x00000001;
	DWT->CTRL |=  DWT_CTRL_CYCCNTENA_Msk; //0x00000001;
	
	DWT->CYCCNT = 0;
	
	__asm volatile ("NOP");
	__asm volatile ("NOP");
	__asm volatile ("NOP");
	
	return (DWT->CYCCNT) ? 0 : 1;
}

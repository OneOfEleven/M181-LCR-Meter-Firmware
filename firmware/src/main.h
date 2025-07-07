/**
 ******************************************************************************
 * @file           : main.h
 * @brief          : Main program header
 ******************************************************************************
 * @attention
 * Developed by: Jaishankar M
 * Re-worked by: OneOfEleven
 ******************************************************************************
 */

#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
	extern "C"
	{
#endif

#include <stdint.h>
#include <stdbool.h>
#include <math.h>

#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_tim.h"

#ifndef DEBUG
	#define USE_IWDG                               // useful to reset the CPU if something locks up etc
#endif

#define FW_VERSION                   0.25

#define ARRAY_SIZE(x)               (sizeof(x) / sizeof((x)[0]))

#define RAD_TO_DEG                  ((float)(180.0 / M_PI))
#define DEG_TO_RAD                  ((float)(M_PI / 180.0))

#define SQR(x)                      ((x) * (x))

//#define UART_BAUDRATE              115200
#define UART_BAUDRATE                230400         // maximum rate CH340N USB chip can do

#define PACKET_MARKER                0x19621996     // 32-bit marker to indicate 'start of packet' for the receiver (windows GUI etc)

#define DMA_ADC_DATA_LENGTH          128            // must be 2^n (2, 4, 8 .. 1024 etc)

#define DEFAULT_ADC_AVERAGE_COUNT    24             // must be >= 1      number of ADC blocks to average     1 = just one block = no averaging

#define MODE_SWITCH_BLOCK_WAIT       10             // number of sample blocks to wait before saving blocks after switching mode - to overcome design floor

#define HIGH                         GPIO_PIN_SET
#define LOW                          GPIO_PIN_RESET

#define ADC1_Pin                     GPIO_PIN_0
#define ADC1_pin_GPIO_Port           GPIOA

#define ADC2_Pin                     GPIO_PIN_1
#define ADC2_pin_GPIO_Port           GPIOA

#define TP21_Pin                     GPIO_PIN_2
#define TP21_pin_GPIO_Port           GPIOA

#define SW_I2C_SDA_Pin               GPIO_PIN_3
#define SW_I2C_SDA_GPIO_Port         GPIOA

#define SW_I2C_SCL_Pin               GPIO_PIN_4
#define SW_I2C_SCL_GPIO_Port         GPIOA

#define LED_Pin                      GPIO_PIN_5
#define LED_pin_GPIO_Port            GPIOA

#define GS_Pin                       GPIO_PIN_6
#define GS_pin_GPIO_Port             GPIOA

#define VI_Pin                       GPIO_PIN_7
#define VI_pin_GPIO_Port             GPIOA

#define TP22_Pin                     GPIO_PIN_8
#define TP22_pin_GPIO_Port           GPIOA

#define UART1_TXD_Pin                GPIO_PIN_9
#define UART1_TXD_GPIO_Port          GPIOA

#define UART1_RXD_Pin                GPIO_PIN_10
#define UART1_RXD_GPIO_Port          GPIOA

#define SWDIO_Pin                    GPIO_PIN_13
#define SWDIO_GPIO_Port              GPIOA

#define SWCLK_Pin                    GPIO_PIN_14
#define SWCLK_GPIO_Port              GPIOA

#define DA0_Pin                      GPIO_PIN_0
#define DA0_pin_GPIO_Port            GPIOB

#define DA1_Pin                      GPIO_PIN_1
#define DA1_pin_GPIO_Port            GPIOB

#define DA2_Pin                      GPIO_PIN_2
#define DA2_pin_GPIO_Port            GPIOB

#define DA3_Pin                      GPIO_PIN_3
#define DA3_pin_GPIO_Port            GPIOB

#define DA4_Pin                      GPIO_PIN_4
#define DA4_pin_GPIO_Port            GPIOB

#define DA5_Pin                      GPIO_PIN_5
#define DA5_pin_GPIO_Port            GPIOB

#define DA6_Pin                      GPIO_PIN_6
#define DA6_pin_GPIO_Port            GPIOB

#define DA7_Pin                      GPIO_PIN_7
#define DA7_pin_GPIO_Port            GPIOB

#define BUTT_HOLD_Pin                GPIO_PIN_13
#define BUTT_HOLD_GPIO_Port          GPIOB

#define BUTT_SP_Pin                  GPIO_PIN_14
#define BUTT_SP_GPIO_Port            GPIOB

#define BUTT_RCL_Pin                 GPIO_PIN_15
#define BUTT_RCL_GPIO_Port           GPIOB

enum {
	LCR_MODE_INDUCTANCE = 0,
	LCR_MODE_CAPACITANCE,
	LCR_MODE_RESISTANCE,
	LCR_MODE_TAN_DELTA
};

enum {
	MODE_VOLT_LO_GAIN = 0,
	MODE_AMP_LO_GAIN,
	MODE_VOLT_HI_GAIN,
	MODE_AMP_HI_GAIN,
	MODE_DONE
};

typedef struct
{
	uint16_t     set_freq;
	unsigned int led_state;
	unsigned int vi_measure_mode;
	float        rms_voltage;
	float        rms_afc_volt;
	float        rms_current;
	float        rms_afc_current;
	float        voltage_phase;
	float        current_phase;
	float        vi_phase;
	float        capacitance;
	float        inductance;
	float        resistance;
	int          unit_capacitance;
	int          unit_inductance;
	int          unit_resistance;
	int          unit_esr;
	float        impedance;
	float        esr;
	float        tan_delta;
	float        qf;
	bool         uart_all_print_dso;
	unsigned int lcr_mode;
} t_system_data;

void Error_Handler(void);

void NMI_Handler(void);
void HardFault_Handler(void);
void MemManage_Handler(void);
void BusFault_Handler(void);
void UsageFault_Handler(void);
void SVC_Handler(void);
void DebugMon_Handler(void);
void PendSV_Handler(void);
void SysTick_Handler(void);
void DMA1_Channel1_IRQHandler(void);
void DMA1_Channel4_IRQHandler(void);
void USART1_IRQHandler(void);
void TIM2_IRQHandler(void);
void TIM3_IRQHandler(void);

#ifdef __cplusplus
	}
#endif

#endif

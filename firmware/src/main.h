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
//#include <stdbool.h>
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

//#define MATLAB_SERIAL                             // use this to send UART data as ascii text (rather than binary packets)

//#define UART_BAUDRATE                115200
//#define UART_BAUDRATE                230400
//#define UART_BAUDRATE                460800
#define UART_BAUDRATE                921600
//#define UART_BAUDRATE                1843200      // max rate the CH340N can do

//#define UART_BIG_ENDIAN                           // use this if you want your uart data sent in big endian order

#define PACKET_MARKER                0x19621996     // 32-bit marker to indicate 'start of packet' for the receiver (windows GUI etc)

#define DMA_ADC_DATA_LENGTH          128            // 2^n

#define MODE_SWITCH_BLOCK_WAIT_SHORT 4              // number of sample blocks to wait after switching modes before saving them
#define MODE_SWITCH_BLOCK_WAIT_LONG  10             // number of sample blocks to wait after switching modes before saving them

#define DEFAULT_ADC_AVERAGE_COUNT    32             // must be >= 1      number of ADC blocks to average     1 = just one block = no averaging

//#define GOERTZEL_FILTER_LENGTH     0                          // don't Goertzel filter
#define GOERTZEL_FILTER_LENGTH    (DMA_ADC_DATA_LENGTH / 2)   // one sine cycle filter length, less filtering, but quicker than full filtering
//#define GOERTZEL_FILTER_LENGTH     DMA_ADC_DATA_LENGTH        // max length filtering (nice but takes more time)

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

// operating mode
enum {
	OP_MODE_MEASURING = 0,
	OP_MODE_OPEN_ZEROING,
	OP_MODE_SHORT_ZEROING
};

// LCR mode
enum {
	LCR_MODE_INDUCTANCE = 0,
	LCR_MODE_CAPACITANCE,
	LCR_MODE_RESISTANCE,
	LCR_MODE_TAN_DELTA
};

// VI mode
enum {
	VI_MODE_VOLT_LO_GAIN = 0,
	VI_MODE_AMP_LO_GAIN,
	VI_MODE_VOLT_HI_GAIN,
	VI_MODE_AMP_HI_GAIN,
	VI_MODE_DONE
};

// this array will be stored in flash (emulated EEPROM)
#pragma pack(push, 1)
typedef struct {
	uint16_t     set_freq;
	unsigned int lcr_mode;
	uint8_t      uart_all_print_dso;

	struct {
		float    mag_rms[4];
		float    phase_deg[4];
		uint8_t  done;
	} open_zero;

	struct {
		float    mag_rms[4];
		float    phase_deg[4];
		uint8_t  done;
	} short_zero;

} t_settings;
#pragma pack(pop)

typedef struct {
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
void TIM3_IRQHandler(void);

#ifdef __cplusplus
	}
#endif

#endif

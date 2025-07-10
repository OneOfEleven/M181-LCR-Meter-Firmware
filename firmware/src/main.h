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

#define SETTINGS_MARKER              0x19961962     // 32-bit marker to indicate 'start of settings'

#define DMA_ADC_DATA_LENGTH          128            // 2^n

#define MODE_SWITCH_BLOCK_WAIT_SHORT 4              // number of sample blocks to wait after switching modes before saving them
#define MODE_SWITCH_BLOCK_WAIT_LONG  10             //   "         "      "         "

#define DEFAULT_ADC_AVERAGE_COUNT    32             // must be >= 1      number of ADC blocks to average     1 = just one block = no averaging

//#define GOERTZEL_FILTER_LENGTH     0                          // don't Goertzel filter
#define GOERTZEL_FILTER_LENGTH      (DMA_ADC_DATA_LENGTH / 2)   // one sine cycle filter length, less filtering, but quicker than full filtering
//#define GOERTZEL_FILTER_LENGTH     DMA_ADC_DATA_LENGTH        // max length filtering (nice but takes more time)

#define ZEROING_COUNT                10             // averaging length to use for open/short zeroing

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

// *******************************

#define ADDR_FLASH_PAGE_0     ((uint32_t)0x08000000) /* Base @ of Page 0, 1 Kbytes */
#define ADDR_FLASH_PAGE_1     ((uint32_t)0x08000400) /* Base @ of Page 1, 1 Kbytes */
#define ADDR_FLASH_PAGE_2     ((uint32_t)0x08000800) /* Base @ of Page 2, 1 Kbytes */
#define ADDR_FLASH_PAGE_3     ((uint32_t)0x08000C00) /* Base @ of Page 3, 1 Kbytes */
#define ADDR_FLASH_PAGE_4     ((uint32_t)0x08001000) /* Base @ of Page 4, 1 Kbytes */
#define ADDR_FLASH_PAGE_5     ((uint32_t)0x08001400) /* Base @ of Page 5, 1 Kbytes */
#define ADDR_FLASH_PAGE_6     ((uint32_t)0x08001800) /* Base @ of Page 6, 1 Kbytes */
#define ADDR_FLASH_PAGE_7     ((uint32_t)0x08001C00) /* Base @ of Page 7, 1 Kbytes */
#define ADDR_FLASH_PAGE_8     ((uint32_t)0x08002000) /* Base @ of Page 8, 1 Kbytes */
#define ADDR_FLASH_PAGE_9     ((uint32_t)0x08002400) /* Base @ of Page 9, 1 Kbytes */
#define ADDR_FLASH_PAGE_10    ((uint32_t)0x08002800) /* Base @ of Page 10, 1 Kbytes */
#define ADDR_FLASH_PAGE_11    ((uint32_t)0x08002C00) /* Base @ of Page 11, 1 Kbytes */
#define ADDR_FLASH_PAGE_12    ((uint32_t)0x08003000) /* Base @ of Page 12, 1 Kbytes */
#define ADDR_FLASH_PAGE_13    ((uint32_t)0x08003400) /* Base @ of Page 13, 1 Kbytes */
#define ADDR_FLASH_PAGE_14    ((uint32_t)0x08003800) /* Base @ of Page 14, 1 Kbytes */
#define ADDR_FLASH_PAGE_15    ((uint32_t)0x08003C00) /* Base @ of Page 15, 1 Kbytes */
#define ADDR_FLASH_PAGE_16    ((uint32_t)0x08004000) /* Base @ of Page 16, 1 Kbytes */
#define ADDR_FLASH_PAGE_17    ((uint32_t)0x08004400) /* Base @ of Page 17, 1 Kbytes */
#define ADDR_FLASH_PAGE_18    ((uint32_t)0x08004800) /* Base @ of Page 18, 1 Kbytes */
#define ADDR_FLASH_PAGE_19    ((uint32_t)0x08004C00) /* Base @ of Page 19, 1 Kbytes */
#define ADDR_FLASH_PAGE_20    ((uint32_t)0x08005000) /* Base @ of Page 20, 1 Kbytes */
#define ADDR_FLASH_PAGE_21    ((uint32_t)0x08005400) /* Base @ of Page 21, 1 Kbytes */
#define ADDR_FLASH_PAGE_22    ((uint32_t)0x08005800) /* Base @ of Page 22, 1 Kbytes */
#define ADDR_FLASH_PAGE_23    ((uint32_t)0x08005C00) /* Base @ of Page 23, 1 Kbytes */
#define ADDR_FLASH_PAGE_24    ((uint32_t)0x08006000) /* Base @ of Page 24, 1 Kbytes */
#define ADDR_FLASH_PAGE_25    ((uint32_t)0x08006400) /* Base @ of Page 25, 1 Kbytes */
#define ADDR_FLASH_PAGE_26    ((uint32_t)0x08006800) /* Base @ of Page 26, 1 Kbytes */
#define ADDR_FLASH_PAGE_27    ((uint32_t)0x08006C00) /* Base @ of Page 27, 1 Kbytes */
#define ADDR_FLASH_PAGE_28    ((uint32_t)0x08007000) /* Base @ of Page 28, 1 Kbytes */
#define ADDR_FLASH_PAGE_29    ((uint32_t)0x08007400) /* Base @ of Page 29, 1 Kbytes */
#define ADDR_FLASH_PAGE_30    ((uint32_t)0x08007800) /* Base @ of Page 30, 1 Kbytes */
#define ADDR_FLASH_PAGE_31    ((uint32_t)0x08007C00) /* Base @ of Page 31, 1 Kbytes */
#define ADDR_FLASH_PAGE_32    ((uint32_t)0x08008000) /* Base @ of Page 32, 1 Kbytes */
#define ADDR_FLASH_PAGE_33    ((uint32_t)0x08008400) /* Base @ of Page 33, 1 Kbytes */
#define ADDR_FLASH_PAGE_34    ((uint32_t)0x08008800) /* Base @ of Page 34, 1 Kbytes */
#define ADDR_FLASH_PAGE_35    ((uint32_t)0x08008C00) /* Base @ of Page 35, 1 Kbytes */
#define ADDR_FLASH_PAGE_36    ((uint32_t)0x08009000) /* Base @ of Page 36, 1 Kbytes */
#define ADDR_FLASH_PAGE_37    ((uint32_t)0x08009400) /* Base @ of Page 37, 1 Kbytes */
#define ADDR_FLASH_PAGE_38    ((uint32_t)0x08009800) /* Base @ of Page 38, 1 Kbytes */
#define ADDR_FLASH_PAGE_39    ((uint32_t)0x08009C00) /* Base @ of Page 39, 1 Kbytes */
#define ADDR_FLASH_PAGE_40    ((uint32_t)0x0800A000) /* Base @ of Page 40, 1 Kbytes */
#define ADDR_FLASH_PAGE_41    ((uint32_t)0x0800A400) /* Base @ of Page 41, 1 Kbytes */
#define ADDR_FLASH_PAGE_42    ((uint32_t)0x0800A800) /* Base @ of Page 42, 1 Kbytes */
#define ADDR_FLASH_PAGE_43    ((uint32_t)0x0800AC00) /* Base @ of Page 43, 1 Kbytes */
#define ADDR_FLASH_PAGE_44    ((uint32_t)0x0800B000) /* Base @ of Page 44, 1 Kbytes */
#define ADDR_FLASH_PAGE_45    ((uint32_t)0x0800B400) /* Base @ of Page 45, 1 Kbytes */
#define ADDR_FLASH_PAGE_46    ((uint32_t)0x0800B800) /* Base @ of Page 46, 1 Kbytes */
#define ADDR_FLASH_PAGE_47    ((uint32_t)0x0800BC00) /* Base @ of Page 47, 1 Kbytes */
#define ADDR_FLASH_PAGE_48    ((uint32_t)0x0800C000) /* Base @ of Page 48, 1 Kbytes */
#define ADDR_FLASH_PAGE_49    ((uint32_t)0x0800C400) /* Base @ of Page 49, 1 Kbytes */
#define ADDR_FLASH_PAGE_50    ((uint32_t)0x0800C800) /* Base @ of Page 50, 1 Kbytes */
#define ADDR_FLASH_PAGE_51    ((uint32_t)0x0800CC00) /* Base @ of Page 51, 1 Kbytes */
#define ADDR_FLASH_PAGE_52    ((uint32_t)0x0800D000) /* Base @ of Page 52, 1 Kbytes */
#define ADDR_FLASH_PAGE_53    ((uint32_t)0x0800D400) /* Base @ of Page 53, 1 Kbytes */
#define ADDR_FLASH_PAGE_54    ((uint32_t)0x0800D800) /* Base @ of Page 54, 1 Kbytes */
#define ADDR_FLASH_PAGE_55    ((uint32_t)0x0800DC00) /* Base @ of Page 55, 1 Kbytes */
#define ADDR_FLASH_PAGE_56    ((uint32_t)0x0800E000) /* Base @ of Page 56, 1 Kbytes */
#define ADDR_FLASH_PAGE_57    ((uint32_t)0x0800E400) /* Base @ of Page 57, 1 Kbytes */
#define ADDR_FLASH_PAGE_58    ((uint32_t)0x0800E800) /* Base @ of Page 58, 1 Kbytes */
#define ADDR_FLASH_PAGE_59    ((uint32_t)0x0800EC00) /* Base @ of Page 59, 1 Kbytes */
#define ADDR_FLASH_PAGE_60    ((uint32_t)0x0800F000) /* Base @ of Page 60, 1 Kbytes */
#define ADDR_FLASH_PAGE_61    ((uint32_t)0x0800F400) /* Base @ of Page 61, 1 Kbytes */
#define ADDR_FLASH_PAGE_62    ((uint32_t)0x0800F800) /* Base @ of Page 62, 1 Kbytes */
#define ADDR_FLASH_PAGE_63    ((uint32_t)0x0800FC00) /* Base @ of Page 63, 1 Kbytes */
#define ADDR_FLASH_PAGE_64    ((uint32_t)0x08010000) /* Base @ of Page 64, 1 Kbytes */
#define ADDR_FLASH_PAGE_65    ((uint32_t)0x08010400) /* Base @ of Page 65, 1 Kbytes */
#define ADDR_FLASH_PAGE_66    ((uint32_t)0x08010800) /* Base @ of Page 66, 1 Kbytes */
#define ADDR_FLASH_PAGE_67    ((uint32_t)0x08010C00) /* Base @ of Page 67, 1 Kbytes */
#define ADDR_FLASH_PAGE_68    ((uint32_t)0x08011000) /* Base @ of Page 68, 1 Kbytes */
#define ADDR_FLASH_PAGE_69    ((uint32_t)0x08011400) /* Base @ of Page 69, 1 Kbytes */
#define ADDR_FLASH_PAGE_70    ((uint32_t)0x08011800) /* Base @ of Page 70, 1 Kbytes */
#define ADDR_FLASH_PAGE_71    ((uint32_t)0x08011C00) /* Base @ of Page 71, 1 Kbytes */
#define ADDR_FLASH_PAGE_72    ((uint32_t)0x08012000) /* Base @ of Page 72, 1 Kbytes */
#define ADDR_FLASH_PAGE_73    ((uint32_t)0x08012400) /* Base @ of Page 73, 1 Kbytes */
#define ADDR_FLASH_PAGE_74    ((uint32_t)0x08012800) /* Base @ of Page 74, 1 Kbytes */
#define ADDR_FLASH_PAGE_75    ((uint32_t)0x08012C00) /* Base @ of Page 75, 1 Kbytes */
#define ADDR_FLASH_PAGE_76    ((uint32_t)0x08013000) /* Base @ of Page 76, 1 Kbytes */
#define ADDR_FLASH_PAGE_77    ((uint32_t)0x08013400) /* Base @ of Page 77, 1 Kbytes */
#define ADDR_FLASH_PAGE_78    ((uint32_t)0x08013800) /* Base @ of Page 78, 1 Kbytes */
#define ADDR_FLASH_PAGE_79    ((uint32_t)0x08013C00) /* Base @ of Page 79, 1 Kbytes */
#define ADDR_FLASH_PAGE_80    ((uint32_t)0x08014000) /* Base @ of Page 80, 1 Kbytes */
#define ADDR_FLASH_PAGE_81    ((uint32_t)0x08014400) /* Base @ of Page 81, 1 Kbytes */
#define ADDR_FLASH_PAGE_82    ((uint32_t)0x08014800) /* Base @ of Page 82, 1 Kbytes */
#define ADDR_FLASH_PAGE_83    ((uint32_t)0x08014C00) /* Base @ of Page 83, 1 Kbytes */
#define ADDR_FLASH_PAGE_84    ((uint32_t)0x08015000) /* Base @ of Page 84, 1 Kbytes */
#define ADDR_FLASH_PAGE_85    ((uint32_t)0x08015400) /* Base @ of Page 85, 1 Kbytes */
#define ADDR_FLASH_PAGE_86    ((uint32_t)0x08015800) /* Base @ of Page 86, 1 Kbytes */
#define ADDR_FLASH_PAGE_87    ((uint32_t)0x08015C00) /* Base @ of Page 87, 1 Kbytes */
#define ADDR_FLASH_PAGE_88    ((uint32_t)0x08016000) /* Base @ of Page 88, 1 Kbytes */
#define ADDR_FLASH_PAGE_89    ((uint32_t)0x08016400) /* Base @ of Page 89, 1 Kbytes */
#define ADDR_FLASH_PAGE_90    ((uint32_t)0x08016800) /* Base @ of Page 90, 1 Kbytes */
#define ADDR_FLASH_PAGE_91    ((uint32_t)0x08016C00) /* Base @ of Page 91, 1 Kbytes */
#define ADDR_FLASH_PAGE_92    ((uint32_t)0x08017000) /* Base @ of Page 92, 1 Kbytes */
#define ADDR_FLASH_PAGE_93    ((uint32_t)0x08017400) /* Base @ of Page 93, 1 Kbytes */
#define ADDR_FLASH_PAGE_94    ((uint32_t)0x08017800) /* Base @ of Page 94, 1 Kbytes */
#define ADDR_FLASH_PAGE_95    ((uint32_t)0x08017C00) /* Base @ of Page 95, 1 Kbytes */
#define ADDR_FLASH_PAGE_96    ((uint32_t)0x08018000) /* Base @ of Page 96, 1 Kbytes */
#define ADDR_FLASH_PAGE_97    ((uint32_t)0x08018400) /* Base @ of Page 97, 1 Kbytes */
#define ADDR_FLASH_PAGE_98    ((uint32_t)0x08018800) /* Base @ of Page 98, 1 Kbytes */
#define ADDR_FLASH_PAGE_99    ((uint32_t)0x08018C00) /* Base @ of Page 99, 1 Kbytes */
#define ADDR_FLASH_PAGE_100   ((uint32_t)0x08019000) /* Base @ of Page 100, 1 Kbytes */
#define ADDR_FLASH_PAGE_101   ((uint32_t)0x08019400) /* Base @ of Page 101, 1 Kbytes */
#define ADDR_FLASH_PAGE_102   ((uint32_t)0x08019800) /* Base @ of Page 102, 1 Kbytes */
#define ADDR_FLASH_PAGE_103   ((uint32_t)0x08019C00) /* Base @ of Page 103, 1 Kbytes */
#define ADDR_FLASH_PAGE_104   ((uint32_t)0x0801A000) /* Base @ of Page 104, 1 Kbytes */
#define ADDR_FLASH_PAGE_105   ((uint32_t)0x0801A400) /* Base @ of Page 105, 1 Kbytes */
#define ADDR_FLASH_PAGE_106   ((uint32_t)0x0801A800) /* Base @ of Page 106, 1 Kbytes */
#define ADDR_FLASH_PAGE_107   ((uint32_t)0x0801AC00) /* Base @ of Page 107, 1 Kbytes */
#define ADDR_FLASH_PAGE_108   ((uint32_t)0x0801B000) /* Base @ of Page 108, 1 Kbytes */
#define ADDR_FLASH_PAGE_109   ((uint32_t)0x0801B400) /* Base @ of Page 109, 1 Kbytes */
#define ADDR_FLASH_PAGE_110   ((uint32_t)0x0801B800) /* Base @ of Page 110, 1 Kbytes */
#define ADDR_FLASH_PAGE_111   ((uint32_t)0x0801BC00) /* Base @ of Page 111, 1 Kbytes */
#define ADDR_FLASH_PAGE_112   ((uint32_t)0x0801C000) /* Base @ of Page 112, 1 Kbytes */
#define ADDR_FLASH_PAGE_113   ((uint32_t)0x0801C400) /* Base @ of Page 113, 1 Kbytes */
#define ADDR_FLASH_PAGE_114   ((uint32_t)0x0801C800) /* Base @ of Page 114, 1 Kbytes */
#define ADDR_FLASH_PAGE_115   ((uint32_t)0x0801CC00) /* Base @ of Page 115, 1 Kbytes */
#define ADDR_FLASH_PAGE_116   ((uint32_t)0x0801D000) /* Base @ of Page 116, 1 Kbytes */
#define ADDR_FLASH_PAGE_117   ((uint32_t)0x0801D400) /* Base @ of Page 117, 1 Kbytes */
#define ADDR_FLASH_PAGE_118   ((uint32_t)0x0801D800) /* Base @ of Page 118, 1 Kbytes */
#define ADDR_FLASH_PAGE_119   ((uint32_t)0x0801DC00) /* Base @ of Page 119, 1 Kbytes */
#define ADDR_FLASH_PAGE_120   ((uint32_t)0x0801E000) /* Base @ of Page 120, 1 Kbytes */
#define ADDR_FLASH_PAGE_121   ((uint32_t)0x0801E400) /* Base @ of Page 121, 1 Kbytes */
#define ADDR_FLASH_PAGE_122   ((uint32_t)0x0801E800) /* Base @ of Page 122, 1 Kbytes */
#define ADDR_FLASH_PAGE_123   ((uint32_t)0x0801EC00) /* Base @ of Page 123, 1 Kbytes */
#define ADDR_FLASH_PAGE_124   ((uint32_t)0x0801F000) /* Base @ of Page 124, 1 Kbytes */
#define ADDR_FLASH_PAGE_125   ((uint32_t)0x0801F400) /* Base @ of Page 125, 1 Kbytes */
#define ADDR_FLASH_PAGE_126   ((uint32_t)0x0801F800) /* Base @ of Page 126, 1 Kbytes */
#define ADDR_FLASH_PAGE_127   ((uint32_t)0x0801FC00) /* Base @ of Page 127, 1 Kbytes */

#define PAGE_SIZE              (uint32_t)FLASH_PAGE_SIZE

#define EEPROM_START_ADDRESS  ((uint32_t)ADDR_FLASH_PAGE_63)
//#define EEPROM_START_ADDRESS  ((uint32_t)ADDR_FLASH_PAGE_126)

// *******************************

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

// this structure will be stored in flash (emulated EEPROM)
//
// MUST keep this to a multiple of 32-bits in size
//
#pragma pack(push, 1)
typedef struct {
	uint32_t     marker;              // settings marker
	uint32_t     seq_num;             // used for wear leveling

	uint16_t     measuring_Hz;        // the sinewave measurement frequency (100Hz, 00Hz or 1Khz)
	uint8_t      lcr_mode;            // the mode the user is using
	uint8_t      uart_all_print_dso;  // set to '1' if to send all sampled ADC data down the serial port

	// open zeroing results
	struct {
		float    mag_rms[8];          // averaged RMS magnitude values for each VI mode
		float    phase_deg[8];        // averaged phase values for each VI mode
		uint8_t  done;                // set to '1' after the user has done this calibration step, otherwise '0'
		uint8_t  padding[3];
	} open_zero;

	// short zeroing results
	struct {
		float    mag_rms[8];          // averaged RMS magnitude values for each VI mode
		float    phase_deg[8];        // averaged phase values for each VI mode
		uint8_t  done;                // set to '1' after the user has done this calibration step, otherwise '0'
		uint8_t  padding[3];
	} short_zero;

	uint8_t      padding[2];

	uint16_t     crc;                 // CRC value for the entire structure. CRC computed with this set to '0'
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

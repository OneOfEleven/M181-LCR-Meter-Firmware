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

#include <math.h>

#include "stm32f1xx_hal.h"

#ifndef DEBUG
	#define USE_IWDG                               // useful to reset the CPU if something locks up etc
#endif

#ifndef M_PI
    #define M_PI                     3.14159265358979323846264338327950288
#endif

#define FW_VERSION                   0.3

#define ARRAY_SIZE(x)               (sizeof(x) / sizeof((x)[0]))

#define RAD_TO_DEG                  ((float)(180.0 / M_PI))
#define DEG_TO_RAD                  ((float)(M_PI / 180.0))

#define SQR(x)                      ((x) * (x))

//#define MATLAB_SERIAL                             // use this to send UART data as ascii text (rather than binary packets)

//#define UART_BAUDRATE              115200
//#define UART_BAUDRATE              230400
//#define UART_BAUDRATE              460800
#define UART_BAUDRATE                921600
//#define UART_BAUDRATE              1843200      // max rate the CH340N can do

//#define UART_BIG_ENDIAN                           // use this if you want your uart data sent in big endian order

#define PACKET_MARKER                0x19621996     // 32-bit marker to indicate 'start of packet' for the receiver (windows GUI etc)

#define SETTINGS_MARKER              0x19961962     // 32-bit marker to indicate 'start of settings'

#define ADC_DATA_LENGTH              128            // 2^n

#define DUAL_ADC_MODE                               // comment out to use single ADC dual channel mode

#define SERIES_RESISTOR              1000           // series impedance of the DUT feed voltage

#define MODE_SWITCH_BLOCK_WAIT_SHORT 4              // number of sample blocks to wait after switching modes before saving them
#define MODE_SWITCH_BLOCK_WAIT_LONG  10             //   "         "      "         "

#define DEFAULT_ADC_AVERAGE_COUNT    32             // 32  must be >= 1      number of ADC blocks to average     1 = just one block = no averaging

//#define GOERTZEL_FILTER_LENGTH     0                      // don't Goertzel filter
//#define GOERTZEL_FILTER_LENGTH     (ADC_DATA_LENGTH / 2)  // one sine cycle filter length, less filtering, but quicker than full filtering
#define GOERTZEL_FILTER_LENGTH       ADC_DATA_LENGTH        // max length filtering (takes slightly longer)

#define CALIBRATE_COUNT              10             // number of results to average when doing the open/short calibration

#define SAVE_SETTINGS_MS             5000           // ms we wait till we save new settings (helps reduce number of flash writes)

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

#define ADDR_FLASH_PAGE_0            0x08000000
#define ADDR_FLASH_PAGE_1            0x08000400
#define ADDR_FLASH_PAGE_2            0x08000800
#define ADDR_FLASH_PAGE_3            0x08000C00
#define ADDR_FLASH_PAGE_4            0x08001000
#define ADDR_FLASH_PAGE_5            0x08001400
#define ADDR_FLASH_PAGE_6            0x08001800
#define ADDR_FLASH_PAGE_7            0x08001C00
#define ADDR_FLASH_PAGE_8            0x08002000
#define ADDR_FLASH_PAGE_9            0x08002400
#define ADDR_FLASH_PAGE_10           0x08002800
#define ADDR_FLASH_PAGE_11           0x08002C00
#define ADDR_FLASH_PAGE_12           0x08003000
#define ADDR_FLASH_PAGE_13           0x08003400
#define ADDR_FLASH_PAGE_14           0x08003800
#define ADDR_FLASH_PAGE_15           0x08003C00
#define ADDR_FLASH_PAGE_16           0x08004000
#define ADDR_FLASH_PAGE_17           0x08004400
#define ADDR_FLASH_PAGE_18           0x08004800
#define ADDR_FLASH_PAGE_19           0x08004C00
#define ADDR_FLASH_PAGE_20           0x08005000
#define ADDR_FLASH_PAGE_21           0x08005400
#define ADDR_FLASH_PAGE_22           0x08005800
#define ADDR_FLASH_PAGE_23           0x08005C00
#define ADDR_FLASH_PAGE_24           0x08006000
#define ADDR_FLASH_PAGE_25           0x08006400
#define ADDR_FLASH_PAGE_26           0x08006800
#define ADDR_FLASH_PAGE_27           0x08006C00
#define ADDR_FLASH_PAGE_28           0x08007000
#define ADDR_FLASH_PAGE_29           0x08007400
#define ADDR_FLASH_PAGE_30           0x08007800
#define ADDR_FLASH_PAGE_31           0x08007C00
#define ADDR_FLASH_PAGE_32           0x08008000
#define ADDR_FLASH_PAGE_33           0x08008400
#define ADDR_FLASH_PAGE_34           0x08008800
#define ADDR_FLASH_PAGE_35           0x08008C00
#define ADDR_FLASH_PAGE_36           0x08009000
#define ADDR_FLASH_PAGE_37           0x08009400
#define ADDR_FLASH_PAGE_38           0x08009800
#define ADDR_FLASH_PAGE_39           0x08009C00
#define ADDR_FLASH_PAGE_40           0x0800A000
#define ADDR_FLASH_PAGE_41           0x0800A400
#define ADDR_FLASH_PAGE_42           0x0800A800
#define ADDR_FLASH_PAGE_43           0x0800AC00
#define ADDR_FLASH_PAGE_44           0x0800B000
#define ADDR_FLASH_PAGE_45           0x0800B400
#define ADDR_FLASH_PAGE_46           0x0800B800
#define ADDR_FLASH_PAGE_47           0x0800BC00
#define ADDR_FLASH_PAGE_48           0x0800C000
#define ADDR_FLASH_PAGE_49           0x0800C400
#define ADDR_FLASH_PAGE_50           0x0800C800
#define ADDR_FLASH_PAGE_51           0x0800CC00
#define ADDR_FLASH_PAGE_52           0x0800D000
#define ADDR_FLASH_PAGE_53           0x0800D400
#define ADDR_FLASH_PAGE_54           0x0800D800
#define ADDR_FLASH_PAGE_55           0x0800DC00
#define ADDR_FLASH_PAGE_56           0x0800E000
#define ADDR_FLASH_PAGE_57           0x0800E400
#define ADDR_FLASH_PAGE_58           0x0800E800
#define ADDR_FLASH_PAGE_59           0x0800EC00
#define ADDR_FLASH_PAGE_60           0x0800F000
#define ADDR_FLASH_PAGE_61           0x0800F400
#define ADDR_FLASH_PAGE_62           0x0800F800
#define ADDR_FLASH_PAGE_63           0x0800FC00
#define ADDR_FLASH_PAGE_64           0x08010000
#define ADDR_FLASH_PAGE_65           0x08010400
#define ADDR_FLASH_PAGE_66           0x08010800
#define ADDR_FLASH_PAGE_67           0x08010C00
#define ADDR_FLASH_PAGE_68           0x08011000
#define ADDR_FLASH_PAGE_69           0x08011400
#define ADDR_FLASH_PAGE_70           0x08011800
#define ADDR_FLASH_PAGE_71           0x08011C00
#define ADDR_FLASH_PAGE_72           0x08012000
#define ADDR_FLASH_PAGE_73           0x08012400
#define ADDR_FLASH_PAGE_74           0x08012800
#define ADDR_FLASH_PAGE_75           0x08012C00
#define ADDR_FLASH_PAGE_76           0x08013000
#define ADDR_FLASH_PAGE_77           0x08013400
#define ADDR_FLASH_PAGE_78           0x08013800
#define ADDR_FLASH_PAGE_79           0x08013C00
#define ADDR_FLASH_PAGE_80           0x08014000
#define ADDR_FLASH_PAGE_81           0x08014400
#define ADDR_FLASH_PAGE_82           0x08014800
#define ADDR_FLASH_PAGE_83           0x08014C00
#define ADDR_FLASH_PAGE_84           0x08015000
#define ADDR_FLASH_PAGE_85           0x08015400
#define ADDR_FLASH_PAGE_86           0x08015800
#define ADDR_FLASH_PAGE_87           0x08015C00
#define ADDR_FLASH_PAGE_88           0x08016000
#define ADDR_FLASH_PAGE_89           0x08016400
#define ADDR_FLASH_PAGE_90           0x08016800
#define ADDR_FLASH_PAGE_91           0x08016C00
#define ADDR_FLASH_PAGE_92           0x08017000
#define ADDR_FLASH_PAGE_93           0x08017400
#define ADDR_FLASH_PAGE_94           0x08017800
#define ADDR_FLASH_PAGE_95           0x08017C00
#define ADDR_FLASH_PAGE_96           0x08018000
#define ADDR_FLASH_PAGE_97           0x08018400
#define ADDR_FLASH_PAGE_98           0x08018800
#define ADDR_FLASH_PAGE_99           0x08018C00
#define ADDR_FLASH_PAGE_100          0x08019000
#define ADDR_FLASH_PAGE_101          0x08019400
#define ADDR_FLASH_PAGE_102          0x08019800
#define ADDR_FLASH_PAGE_103          0x08019C00
#define ADDR_FLASH_PAGE_104          0x0801A000
#define ADDR_FLASH_PAGE_105          0x0801A400
#define ADDR_FLASH_PAGE_106          0x0801A800
#define ADDR_FLASH_PAGE_107          0x0801AC00
#define ADDR_FLASH_PAGE_108          0x0801B000
#define ADDR_FLASH_PAGE_109          0x0801B400
#define ADDR_FLASH_PAGE_110          0x0801B800
#define ADDR_FLASH_PAGE_111          0x0801BC00
#define ADDR_FLASH_PAGE_112          0x0801C000
#define ADDR_FLASH_PAGE_113          0x0801C400
#define ADDR_FLASH_PAGE_114          0x0801C800
#define ADDR_FLASH_PAGE_115          0x0801CC00
#define ADDR_FLASH_PAGE_116          0x0801D000
#define ADDR_FLASH_PAGE_117          0x0801D400
#define ADDR_FLASH_PAGE_118          0x0801D800
#define ADDR_FLASH_PAGE_119          0x0801DC00
#define ADDR_FLASH_PAGE_120          0x0801E000
#define ADDR_FLASH_PAGE_121          0x0801E400
#define ADDR_FLASH_PAGE_122          0x0801E800
#define ADDR_FLASH_PAGE_123          0x0801EC00
#define ADDR_FLASH_PAGE_124          0x0801F000
#define ADDR_FLASH_PAGE_125          0x0801F400
#define ADDR_FLASH_PAGE_126          0x0801F800
#define ADDR_FLASH_PAGE_127          0x0801FC00

#define PAGE_SIZE                    FLASH_PAGE_SIZE

#define END_ADDRESS                 (ADDR_FLASH_PAGE_63  + PAGE_SIZE)    // STM32F103C8  64k
//#define END_ADDRESS               (ADDR_FLASH_PAGE_127 + PAGE_SIZE)    // STM32F103CB  128k

// 4k seems OK to snipe
#define EEPROM_START_ADDRESS         ADDR_FLASH_PAGE_60                  // STM32F103C8  64k
//#define EEPROM_START_ADDRESS       ADDR_FLASH_PAGE_124                 // STM32F103CB  128k

#define EEPROM_END_ADDRESS           END_ADDRESS

// defined in linker script "STM32F103C8_FLASH.ld"
//extern uint32_t _eeprom_address;
//extern uint32_t _eeprom_length;

// *******************************

// operating mode
enum {
	OP_MODE_MEASURING = 0,
	OP_MODE_OPEN_PROBE_CALIBRATION,
	OP_MODE_SHORTED_PROBE_CALIBRATION
};

// LCR mode
enum {
	LCR_MODE_INDUCTANCE = 0,
	LCR_MODE_CAPACITANCE,
	LCR_MODE_RESISTANCE,
	LCR_MODE_AUTO                     // TODO: add code to auto detect if DUT is a cap, ind or res
};

// VI mode
enum {
	VI_MODE_VOLT_LO_GAIN = 0,
	VI_MODE_AMP_LO_GAIN,
	VI_MODE_VOLT_HI_GAIN,
	VI_MODE_AMP_HI_GAIN,
	VI_MODE_DONE
};

#define SETTING_FLAG_UART_DSO    (1u << 0)
#define SETTING_FLAG_PARALLEL    (1u << 1)
#define SETTING_FLAG_HOLD        (1u << 2)

// this structure will be stored in flash (emulated EEPROM) so as to remember various things for the user
//
#pragma pack(push, 1)
typedef struct t_settings {
	uint32_t     marker;              // settings marker - so we can find this saved block in flash area

	uint16_t     measurement_Hz;      // the sine wave measurement frequency the user is using
	uint8_t      lcr_mode;            // the LCR mode the user is using
	uint8_t      flags;

	struct {
		float    mag_rms[8];          // averaged RMS magnitude values for each VI mode
		float    phase_deg[8];        // averaged phase values for each VI mode
		uint8_t  done;                // set to '1' after the user has done this calibration step, otherwise '0'
		uint8_t  padding[3];          // just padding to maintain 32-bit alignment
	} open_probe_calibration[2];      // 100Hz and 1kHz results

	struct {
		float    mag_rms[8];          // averaged RMS magnitude values for each VI mode
		float    phase_deg[8];        // averaged phase values for each VI mode
		uint8_t  done;                // set to '1' after the user has done this calibration step, otherwise '0'
		uint8_t  padding[3];          // just padding to maintain 32-bit alignment
	} shorted_probe_calibration[2];   // 100Hz and 1kHz results

	uint8_t      padding[2];          // just padding to maintain 32-bit alignment

	uint16_t     crc;                 // CRC value for the entire structure. CRC computed with this set to '0'
} t_settings;
#pragma pack(pop)

typedef struct t_system_data {
	unsigned int vi_measure_mode;

	float        rms_voltage_adc;
	float        rms_voltage_afc;
	float        rms_current_adc;
	float        rms_current_afc;

	float        impedance;

	float        voltage_phase_deg;
	float        current_phase_deg;

	float        vi_phase_deg;

	struct {
		float    capacitance;
		float    inductance;
		float    resistance;
		float    esr;
		float    tan_delta;
		float    qf;
	} series;

	struct {
		float    capacitance;
		float    inductance;
		float    resistance;
		float    esr;
		float    tan_delta;
		float    qf;
	} parallel;

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

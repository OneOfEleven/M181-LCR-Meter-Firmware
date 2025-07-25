/**
 ******************************************************************************
 * @file           : main.cpp
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 * Developed by: Jaishankar M
 * Re-worked by: OneOfEleven
 ******************************************************************************
 */

#define __BUFSIZ__ 0  // zero length printf() buffer
#include <stdio.h>

#include <string.h>
#include <stdlib.h>   // strtol()
#include <ctype.h>    // tolower()
#include <float.h>    // INF
#include <limits.h>
#include <errno.h>
//#include <complex>

#include "main.h"
#include "delay.h"
#include "stm32_sw_i2c.h"
#include "ssd1306.h"
#include "crc.h"

// ***********************************************************

enum {
	BUTTON_HOLD = 0,
	BUTTON_SP,
	BUTTON_RCL,
	BUTTON_NUM
};

typedef struct {
	GPIO_TypeDef     *gpio_port;       // the GPIO port for the button
	uint32_t          gpio_pin;        // the GPIO pin for the button
	volatile int16_t  debounce;        // counter for debouncing (switches always become more noisy after time)
	volatile uint32_t pressed_ms;      // milli-sec tick when the button was initially pressed
	volatile uint32_t held_ms;         // number of ms the button has/was held pressed for
	volatile uint8_t  released;        // set to '1' when the user releases the button
	volatile uint8_t  processed;       // set to '1' by the exec once it's delt with the new button press
} t_button;

// 16-bit ADC dual channel
typedef struct {
	int16_t adc;
	int16_t afc;
} t_adc_dma_data_16;

// 32-bit ADC dual channel
typedef struct {
	int32_t adc;
	int32_t afc;
} t_adc_dma_data_32;

// complex float number
typedef struct t_complex {
	float real;
	float imag;

	t_complex()
	{
		real = 0;
		imag = 0;
	}

	t_complex(const float _real)
	{
		real = _real;
		imag = 0;
	}

	t_complex(const float _real, const float _imag)
	{
		real = _real;
		imag = _imag;
	}
} t_complex;

// serial binary data packet
#pragma pack(push, 1)
	typedef struct {
		uint32_t marker;
		uint16_t crc;
		//uint8_t  seq_num;
		//uint8_t  data_type;
		//uint16_t data_size;
		float    data[ADC_DATA_LENGTH * 8];
	} t_packet;
#pragma pack(pop)

/*
static const uint16_t omega_7x10[] = {
	0b0001110000,       // 1
	0b0010001000,       // 2
	0b0100000100,       // 3
	0b0100000100,       // 4
	0b0010001000,       // 5
	0b0001010000,       // 6
	0b0001010000,       // 7
	0b0111011100,       // 8
	0b0000000000,       // 9
	0b0000000000        // 10
};
*/
static const uint16_t omega_13x18[] = {
	0b0000000000000,   // 1
	0b0000111110000,   // 2
	0b0001111111000,   // 3
	0b0011100011100,   // 4
	0b0110000000110,   // 5
	0b0110000000110,   // 6
	0b0110000000110,   // 7
	0b0110000000110,   // 8
	0b0110000000110,   // 9
	0b0011000001100,   // 10
	0b0001100011000,   // 11
	0b0001100011000,   // 12
	0b0001100011000,   // 13
	0b0101100011010,   // 14
	0b0111100011110,   // 15
	0b0000000000000,   // 16
	0b0000000000000,   // 17
	0b0000000000000    // 18
};

struct {
	uint8_t por;
	uint8_t pin;
	uint8_t sft;
	uint8_t iwdg;
	uint8_t wwdg;
	uint8_t lpwr;
	uint8_t lsirdy;
} reset_cause = {0};

#ifdef USE_IWDG
	uint32_t          iwdg_timeout_sec = 8;        // 8 second IWDG timeout
	volatile uint32_t iwdg_tick        = 0;
#endif

volatile uint32_t     sys_tick = 0;                // our own system tick value

LL_RCC_ClocksTypeDef  rcc_clocks = {0};            // various CPU clock frequencies

uint32_t              draw_screen_count = 0;
char                  str_buf[32]       = {0};

t_button              button[BUTTON_NUM] = {0};    // each buttons press data

uint16_t              sine_table[ADC_DATA_LENGTH / 2] = {0};  // length is matched with the ADC sampling (length = one sine cycle)

uint16_t              measurement_Hz        = 1000;           // 100 or 1000
float                 measurement_amplitude = 1.0;            // 0.0 = 0%, 1.0 = 100%, -1.0 = 100% phase inverted

uint8_t               op_mode      = OP_MODE_MEASURING;       // the current operaring mode the user is in
uint8_t               initialising = 1;                       // set to '1' pauses the ADC buffer processing (ADC keeps going though)
uint32_t              frames       = 0;                       // just a frame counter

unsigned int          display_hold = 0;                       // '1' to hold/pause the display

// for when the user is running the calibrations
struct {
	int               count;
	float             mag_sum[8];
	t_complex         phase_sum[8];
} calibrate = {0};

unsigned int          volt_gain_sel   = 0;
unsigned int          amp_gain_sel    = 0;
unsigned int          gain_changed    = 0;

float                 high_gain       = 101;
float                 inv_series_ohms = 1.0f / SERIES_RESISTOR_OHMS;

// ADC DMA raw sample buffer
t_adc_dma_data_16     adc_dma_buffer[2][ADC_DATA_LENGTH];      // *2 for DMA double buffering (ADC/DMA is continuously running)

// ADC sample block averaging buffer
// we take several sample blocks and average them together to reduce noise/increase dynamic range
t_adc_dma_data_32     adc_buffer_sum[ADC_DATA_LENGTH] = {0};                        // summing buffer
unsigned int          adc_buffer_sum_count            = 0;                          // number of sums so far done

#pragma pack(push, 1)
float                 adc_data[8][ADC_DATA_LENGTH] = {0};
#pragma pack(pop)

// non-zero if waveform clipping/saturation is detected (per block)
uint8_t               adc_data_clipping[4] = {0};
uint8_t               adc_data_clipped[4]  = {0};

// the computed waveform magnitudes and phases - these are what's used to compute the DUT parameters
float                 mag_rms[8]   = {0};
float                 phase_deg[8] = {0};

// custom HW VI mode sequence order to minimize mode switching time
// because of a HW design floor (takes time for HW to settle after changing the HW GS/VI mode pins)
const unsigned int    vi_measure_mode_table[] = {0, 1, 3, 2};
volatile unsigned int vi_measure_index = 0;
unsigned int          prev_vi_mode     = -1;

// system settings ar stored in flash, so we keep a timer for saving the users settings to reduce flash wear
volatile int          save_settings_timer = -1;              //
t_settings            settings            = {0};             // the users settings

t_system_data         system_data = {0};                     // various results saved in here

uint8_t               tmp_buffer[sizeof(t_complex) * ADC_DATA_LENGTH];   // buffer must be big enough for what uses it
volatile uint8_t      tmp_buffer_in_use = 0;

// for TX'ing binary packets via the serial port
t_packet              tx_packet;

// serial port stuff
struct {
	struct {
		uint8_t  buffer[1024];     // RX raw buffer
		uint32_t buffer_rd;
		uint32_t buffer_wr;
		uint32_t timer;            // timer for resetting these RX buffers is nothing received for a set time

		struct {
			uint8_t  buffer[256];  // RX text line buffer
			uint32_t buffer_wr;
		} line;
	} rx;
/*
	struct {
		uint8_t  buffer[2048];     // TX raw buffer
		uint32_t buffer_rd;
		uint32_t buffer_wr;
		uint32_t timer;
	} tx;
*/
} volatile serial = {0};

// *************************************************************

// reboot the CPU
//
void reboot(void)
{
	__disable_irq();
	__DSB();
	LL_GPIO_SetOutputPin(LED_GPIO_Port, LED_Pin);        // LED on
	NVIC_SystemReset();
	while (1) {}
}

// stop the exec and flash the LED
//
void stop(uint32_t ms = 0)
{
	__disable_irq();
	__DSB();
	ms = (ms == 0) ? 300 : ms;
	while (1)
	{
		LL_GPIO_SetOutputPin(LED_GPIO_Port, LED_Pin);    // LED on
		DWT_Delay_ms(10);
		LL_GPIO_ResetOutputPin(LED_GPIO_Port, LED_Pin);  // LED off
		DWT_Delay_ms(ms);
	}
}

// remove any trailing zeros from a float string
void trim_trailing_zeros(char buf[])
{
	if (buf == NULL)
		return;

	if (strchr(buf, '.') == NULL)
		return;                    // no DP found

	const unsigned int len = strlen(buf);
	if (len == 0)
		return;

	// find the end of the fp number
	unsigned int index = len - 1;
	while (index > 0 && (buf[index] < '0' || buf[index] > '9'))
		index--;

	if (index == 0)
		return;

	// find the 1st fractional '0'
	unsigned int index2 = index++;
	while (index2 > 0 && buf[index2] == '0')
		index2--;

	// drop the DP if it's left on it's own
	if (buf[index2] == '.')
		index2--;

	if (++index2 >= index)
		return;

	// remove trailing zeros (inc the DP if need be)
	memmove(buf + index2, buf + index, len + 1 - (index - index2));
}

// wait untill the serial TX DMA has completed it's send
//
void wait_tx_dma(uint32_t ms = 0)
{
	ms = (ms == 0) ? 200 : ms;
	const uint32_t tick = sys_tick;
	while (LL_DMA_IsEnabledChannel(DMA1, LL_DMA_CHANNEL_4) && (sys_tick - tick) < ms)
		__WFI();
}

// feed the serial TX DMA with more data to send
//
int start_tx_dma(const void *data, const unsigned int size)
{
	if (data == NULL || size == 0)
		return -1;

	if (!LL_USART_IsEnabled(USART1))
		return -2;      // serial/uart port is disabled

	if (LL_DMA_IsEnabledChannel(DMA1, LL_DMA_CHANNEL_4))
		return -3;      // still busy sending

	// clear all TX DMA flags
	LL_DMA_ClearFlag_GI4(DMA1);

	// give the DMA the data and data size details
	LL_DMA_ConfigAddresses(DMA1, LL_DMA_CHANNEL_4, (uint32_t)data, LL_USART_DMA_GetRegAddr(USART1), LL_DMA_DIRECTION_MEMORY_TO_PERIPH);
	LL_DMA_SetDataLength(  DMA1, LL_DMA_CHANNEL_4, size);

	// start sending
	LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_4);

	return 0;    // OK
}

// intercept printf, dprintf etc
//
int _write(int file, char ptr[], int len)
{
	start_tx_dma(ptr, len);
	wait_tx_dma();
	return len;
}

// ***********************************************************

float adc_to_volts(const float raw_adc)
{
	const float adc_ref = 3.3;
	const float volts   = (adc_ref * raw_adc) * (float)(1.0 / 4096);
	return volts;
}

void set_measure_mode_pins(const unsigned int mode)
{
	switch (mode)
	{
		default:
		case VI_MODE_VOLT_LO_GAIN:           // low gain voltage mode
			LL_GPIO_ResetOutputPin(VI_GPIO_Port, VI_Pin);
			LL_GPIO_SetOutputPin(  GS_GPIO_Port, GS_Pin);
			break;

		case VI_MODE_AMP_LO_GAIN:            // low gain current mode
			LL_GPIO_SetOutputPin(  VI_GPIO_Port, VI_Pin);
			LL_GPIO_SetOutputPin(  GS_GPIO_Port, GS_Pin);
			break;

		case VI_MODE_VOLT_HI_GAIN:           // high gain voltage mode
			LL_GPIO_ResetOutputPin(VI_GPIO_Port, VI_Pin);
			LL_GPIO_ResetOutputPin(GS_GPIO_Port, GS_Pin);
			break;

		case VI_MODE_AMP_HI_GAIN:            // high gain current mode
			LL_GPIO_SetOutputPin(  VI_GPIO_Port, VI_Pin);
			LL_GPIO_ResetOutputPin(GS_GPIO_Port, GS_Pin);
			break;
	}
}

// ***********************************************************
// emulated eeprom in flash
//
// includes wear leveling by saving each successive settings block to a different flash address, the previous saves are left alone.
// once the allocated eeprom flash pages become full, they are all erased in one go ready for more saves to occur.

int clear_settings(void)
{	// erase all saved settings in eeprom

	// contain the faulty flash address if a problem occurs whilst the page(s) are being erased
	uint32_t page_error = 0;

	FLASH_EraseInitTypeDef erase_init = {0};
	erase_init.TypeErase   = FLASH_TYPEERASE_PAGES;
	erase_init.PageAddress = EEPROM_START_ADDRESS;
	erase_init.NbPages     = (EEPROM_END_ADDRESS - EEPROM_START_ADDRESS) / PAGE_SIZE;

	// unlock the flash
	if (HAL_OK != HAL_FLASH_Unlock())
	{	// error
		//HAL_FLASH_Lock();
		return -1;
	}

	// erase all allocated eeprom flash pages
	const HAL_StatusTypeDef status = HAL_FLASHEx_Erase(&erase_init, &page_error);
	if (status != HAL_OK)
	{	// error
		HAL_FLASH_Lock();
		return -2;
	}

	// re-lock the flash
	HAL_FLASH_Lock();

	// all seems OK
	return 0;
}

static t_settings temp_settings;

uint32_t find_last_good_settings(void)
{	// find the last valid flash saved settings we did
	//
	// do this by sequencially scanning through the allocated eeprom flash pages looking
	// for the last error-free slot/block of settings we saved

	// best to keep block size to a multiple of 32-bits
	const uint32_t block_size = ((sizeof(t_settings) + sizeof(uint32_t) - 1) / sizeof(uint32_t)) * sizeof(uint32_t);

	uint32_t flash_addr      = EEPROM_START_ADDRESS;  // start of allocated eeprom flash area
	uint32_t flash_addr_good = 0;                     // flash address of last good slot/block of settings we find

	while ((flash_addr + sizeof(t_settings)) <= EEPROM_END_ADDRESS)
	{
		// copy a block from flash to ram
		memcpy(&temp_settings, (void *)flash_addr, sizeof(t_settings));

		// validate those settings by checking for a the correct start marker and correct CRC
		//
		// if good, then remember it's flash address
		//
		if (temp_settings.marker == SETTINGS_MARKER)
		{
			const uint16_t crc1 = temp_settings.crc;
			temp_settings.crc = 0;
			const uint16_t crc2 = CRC16_block(0, &temp_settings, sizeof(t_settings));
			if (crc1 == crc2)
			{
				flash_addr_good = flash_addr;   // marker and CRC appear good, remember where it's located in flash
				flash_addr     += block_size;   // point to the next possible slot/block
			}
			else
			{
				//flash_addr     += block_size; // point to the next possible slot/block
				flash_addr += sizeof(uint32_t); // scan the flash area 32-bit word by 32-bit word
			}
		}
		else
		{
			//flash_addr     += block_size;     // point to the next possible slot/block
			flash_addr += sizeof(uint32_t);     // scan the flash area 32-bit word by 32-bit word
		}
	}

	// return the flash address of the last saved error-free block of settings we found
	// if none found then return NULL
	return flash_addr_good;
}

int read_settings(void)
{	// read settings from flash

	// fetch the flash address of last known saved settings
	const uint32_t flash_addr = find_last_good_settings();
	if (flash_addr == 0)
		return -1;           // no valid settings found

	// found some, copy them into our system settings
	memcpy(&settings, (void *)flash_addr, sizeof(t_settings));

	// all seems OK
	return 0;
}

int write_settings(void)
{	// write settings to flash

	// best to keep block size to a multiple of 32-bits
	const uint32_t block_size = ((sizeof(t_settings) + sizeof(uint32_t) - 1) / sizeof(uint32_t)) * sizeof(uint32_t);

	// use this if flash page erase is required
	FLASH_EraseInitTypeDef erase_init = {0};
	erase_init.TypeErase   = FLASH_TYPEERASE_PAGES;
	erase_init.PageAddress = EEPROM_START_ADDRESS;
	erase_init.NbPages     = (EEPROM_END_ADDRESS - EEPROM_START_ADDRESS) / PAGE_SIZE;
	uint32_t page_error    = 0;

	// add marker and CRC to the current system settings (so we can find them again in flash)
	settings.marker = SETTINGS_MARKER;
	settings.crc    = 0;
	settings.crc    = CRC16_block(0, &settings, sizeof(t_settings));

	if (HAL_OK != HAL_FLASH_Unlock())
	{	// unlock error :(
		//HAL_FLASH_Lock();
		return -1;
	}

	// find where we last saved the previous settings
	uint32_t flash_addr = find_last_good_settings();
	if (flash_addr > 0)
	{	// found them

		// check to see if the settings we are going to save are any different to the previous ones
		if (memcmp((void *)flash_addr, &settings, sizeof(t_settings)) == 0)
		{	// previous saved settings are identical to current, no need to save again
			HAL_FLASH_Lock();
			return 0;
		}

		// point to the next possibly available flash slot/block
		flash_addr += block_size;

		if ((flash_addr + block_size) > EEPROM_END_ADDRESS)
		{	// no more allocated eeprom flash space left to save into, start again

			// erase all allocated eeprom flash pages
			const HAL_StatusTypeDef status = HAL_FLASHEx_Erase(&erase_init, &page_error);
			if (status != HAL_OK)
			{	// error
				HAL_FLASH_Lock();
				return -2;
			}

			flash_addr = EEPROM_START_ADDRESS;   // start from the beginning
		}
	}
	else
	{	// start saving at the beginning of allocated eeprom flash area
		flash_addr = EEPROM_START_ADDRESS;
	}

	{	// see if the chosen flash slot is empty (or not)
		//
		// we can't write to it if it's not suitably empty
		// the flash bits can be set to '0' without erasing the flash page
		// but can't be set to '1' without first doing a full page flash erase - beware, I think some STM32's are the opposite (if you re-use this code)

		const __IO uint16_t *addr = (uint16_t *)flash_addr;   // flash address we want to write too
		//const uint16_t    *set  = (uint16_t *)&settings;    // address of settings
		unsigned int         i    = 0;

		while (i < sizeof(t_settings))
		{
			#if 0
				const uint16_t set_val = *set++;
				if ((*addr++ & set_val) == set_val)           // check the bits that matter
			#else
				if (*addr++ == 0xffff)                        // check all word bits
			#endif
			{	// this flash word seems good for writing too
				i += sizeof(uint16_t);
				continue;
			}

			// can't program this particular flash word without first erasing it
			//
			// check the next possible available flash location

			flash_addr += block_size;

			addr  = (uint16_t *)flash_addr;   // flash address
			//set = (uint16_t *)&settings;    // system settings address
			i     = 0;

			if ((flash_addr + block_size) <= EEPROM_END_ADDRESS)
				continue;                     // check the new slot

			// no more flash space left to save into, start again

			// erase all allocated eeprom flash pages
			const HAL_StatusTypeDef status = HAL_FLASHEx_Erase(&erase_init, &page_error);
			if (status != HAL_OK)
			{	// hmmmm
				HAL_FLASH_Lock();
				return -3;
			}

			flash_addr = EEPROM_START_ADDRESS;   // flash address to write too
			break;
		}
	}

	{	// write the new settings into flash

		const uint16_t *set = (uint16_t *)&settings; // data we want to save to flash

		for (unsigned int i = 0; i < sizeof(t_settings); i += sizeof(uint16_t))
		{
			// write the word into flash
			const HAL_StatusTypeDef status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, flash_addr + i, *set++);
			if (status == HAL_OK)
				continue;           // it worked !

			// no it didunt :(

			// erase all allocated flash pages, then give up
			HAL_FLASHEx_Erase(&erase_init, &page_error);
			HAL_FLASH_Lock();
			return -4;
		}
	}

	// ensure the new flash write is OK
	if (memcmp((void *)flash_addr, &settings, sizeof(t_settings)) != 0)
	{	// error
		//
		// erase all allocated eeprom flash pages, then go do something else
		HAL_FLASHEx_Erase(&erase_init, &page_error);
		HAL_FLASH_Lock();
		return -5;
	}

	HAL_FLASH_Lock();

	// settings saved seem OK !

	save_settings_timer = -1;

	return 0;
}

// ***********************************************************
// Goertzel stuff

typedef struct {
	float freq;
	float coeff;
	float sin;
	float cos;
} t_goertzel;

t_goertzel goertzel = {0};

// feed the supplied samples through the Goertzel filter
//
// this is for the phase detector, this outputs a single I/Q pair
//
//std::complex <float> goertzel_block(const float *samples, const unsigned int len, t_goertzel *g)
t_complex goertzel_block(const float *samples, const unsigned int len, t_goertzel *g)
{
	register float m1 = 0;
	register float m2 = 0;

	for (unsigned int i = len; i > 0; i--)
	{
		register const float m = *samples++ + (g->coeff * m1) - m2;
		m2 = m1;
		m1 = m;
	}

	const float re = (m1 * g->cos) - m2;
	const float im = -m1 * g->sin;

	// correct the output sample amplitude
	const float scale = 2.0f / len;
	return t_complex(re * scale, im * scale);
}

// this one assumes the sample buffer contains an integer number of full sine cycles
// if it doesn't, then do not use this function
//
// this one os for BPF filtering the waveform, the filtered wave is saved in 'out_samples'
//
//int goertzel_wrap(const float *in_samples, std::complex *out_samples, const unsigned int len, const unsigned int g_len, t_goertzel *g)
int goertzel_wrap(const float *in_samples, t_complex *out_samples, const unsigned int len, const unsigned int g_len, t_goertzel *g)
{
	const float scale = 2.0f / g_len;  // for correcting the output amplitude

	for (unsigned int k = 0; k < len; k++)
	{
		register float m1 = 0;
		register float m2 = 0;

		for (register unsigned int i = 0, n = k; i < g_len; i++)
		{
			register const float m = in_samples[n] + (g->coeff * m1) - m2;
			m2 = m1;
			m1 = m;
			if (++n >= len)
				n = 0;
		}

		const float real = (m1 * g->cos) - m2;
		const float imag = -m1 * g->sin;

		// correct the output sample amplitude
		out_samples[k].real = real * scale;
		out_samples[k].imag = imag * scale;

		// exit if a button is pressed
		//
		// servicing user input is paramount
		//
		for (unsigned int b = 0; b < BUTTON_NUM; b++)
			if (button[b].pressed_ms > 0 || button[b].released)
				return -1;
	}

	return 0;  // completed
}

// create the Goertzel filter coeffs
//
void goertzel_init(t_goertzel *g, const float normalized_freq)
{
	const float w  = (float)(2 * M_PI) * normalized_freq;
	const float wr = cosf(w);
	const float wi = sinf(w);

	g->freq  = normalized_freq;
	g->coeff = wr * 2;
	g->cos   = wr;
	g->sin   = wi;
}

// ***********************************************************

char unit_conversion(float *value)
{
	const int   sign = (*value < 0.0f) ? -1 : 1;
	const float val  = fabsf(*value);

//	if (val < 1e-12f)
//	{	// femto
//		*value = (val * 1e15f) * sign;
//		return 'f';
//	}

	if (val < 1e-9f)
	{	// pico
		*value = (val * 1e12f) * sign;
		return 'p';
	}

	if (val < 1e-6f)
	{	// nano
		*value = (val * 1e9f) * sign;
		return 'n';
	}

	if (val < 1e-3f)
	{	// micro
		*value = (val * 1e6f) * sign;
		return 'u';
	}

	if (val < 1e0f)
	{	// milli
		*value = (val * 1e3f) * sign;
		return 'm';
	}

	if (val < 1e3f)
	{	// unit
		return ' ';
	}

	if (val < 1e6f)
	{	// kilo
		*value = (val * 1e-3f) * sign;
		return 'k';
	}

	if (val < 1e9f)
	{	// Mega
		*value = (val * 1e-6f) * sign;
		return 'M';
	}

	// Giga
	*value = (val * 1e-9f) * sign;
	return 'G';
}

void set_measurement_frequency(uint32_t Hz)
{
	// limit the range
	measurement_Hz = (Hz < MEASURE_HZ_MIN) ? MEASURE_HZ_MIN : (Hz > MEASURE_HZ_MAX) ? MEASURE_HZ_MAX : Hz;

	// lower freq = lower amplitude (due to another HW filter design flaw)
	//
	measurement_amplitude = (float)measurement_Hz / 1000;                            // 0.0 ~ 1.0
	measurement_amplitude = sqrtf(sqrtf(sqrtf(measurement_amplitude)));
	measurement_amplitude = (measurement_amplitude < 0.8f) ? 0.8f : (measurement_amplitude > 1.0f) ? 1.0f : measurement_amplitude;

	{	// fill the sine wave look-up table with one complete sine cycle

		const float scale      = measurement_amplitude * (255 * 0.5f);           // 0-255
		const float phase_step = (2.0 * M_PI) / ARRAY_SIZE(sine_table);

		// the DMA still writes 16-bits at a time to the GPIO when the DMA is set to 8-bit mode :(
		// so create a 16-bit table with the upper 8-bits all high
		// see 9.2.4 (page 173) of the stm32f103xx reference manual about the ODR register being WORD ONLY
		for (unsigned int i = 0; i < ARRAY_SIZE(sine_table); i++)
			sine_table[i] = 0xff00 | (uint8_t)floorf(((1.0f + sinf(phase_step * i)) * scale) + 0.5f); // raised sine
	}

	if (measurement_Hz > 0)
	{	// set the timer rate
		const uint32_t timer_rate_Hz = (ADC_DATA_LENGTH / 2) * measurement_Hz;
//		const uint32_t period        = __LL_TIM_CALC_ARR(rcc_clocks.HCLK_Frequency, LL_TIM_GetPrescaler(TIM3), timer_rate_Hz);
		const uint32_t period        = (((rcc_clocks.HCLK_Frequency / (LL_TIM_GetPrescaler(TIM3) + 1)) + (timer_rate_Hz / 2)) / timer_rate_Hz) - 1;
		LL_TIM_SetAutoReload(TIM3, period);
	}
}

#if 1
	float phase_diff(const t_complex c1, const t_complex c2)
	{
		// conj multiply
		const t_complex d = t_complex((c1.real * c2.real) + (c1.imag * c2.imag), (c1.real * c2.imag) - (c1.imag * c2.real));

		// return phase difference in degrees
		return (d.real != 0) ? atan2f(d.imag, d.real) * RAD_TO_DEG : NAN;
	}

	float phase_diff(const float phase_deg_1, const float phase_deg_2)
	{
		const float phi1 = phase_deg_1 * DEG_TO_RAD;
		const float phi2 = phase_deg_2 * DEG_TO_RAD;
		return phase_diff(t_complex(cosf(phi1), sinf(phi1)), t_complex(cosf(phi2), sinf(phi2)));
	}
#else
	float phase_diff(float phase_deg_1, float phase_deg_2)
	{
		while (phase_deg_1 < 0) phase_deg_1 += 360.0f;
		while (phase_deg_2 < 0) phase_deg_2 += 360.0f;

		// return shortest angle difference - clockwise or anti-clockwise
		#if 1
			float deg = (phase_deg_1 - phase_deg_2) + 360.0f + 180.0f;
			while (deg >= 360.0f) deg -= 360.0f;
			//while (deg <    0.0f) deg += 360.0f;
			return deg - 180.0f;
		#else
			return fmodf((phase_deg_1 - phase_deg_2) + 360.0f + 180.0f, 360.0f) - 180.0f;
		#endif
	}
#endif

#ifdef AVERAGE_PHASE
	t_complex phi_table[ADC_DATA_LENGTH];
	uint8_t   phi_table_ready = 0;          // flag to say the table has been created (or not)
#endif

// pass the new ADC samples through the goertzel dft
//
// goertzel output samples are 100% free of any DC offset, also very much cleaned of any out-of-band noise (crucial for following measurements)
//
// input is real
// output is complex (I/Q)
//
int process_Goertzel(void)
{
	// the Goertzel DFT is used for creating I/Q output, phase computation and filtering (if desired)

	// the STM32F103 MCU doesn't have a HW FPU, so full length filtering takes quite a time :(
	//
	// a better (drop in replacement) MCU to use would be the STM32F303CBT6 (or others), it has a HW FPU which greatly improves FP ops
	// it also has a dual 12-DAC, but the two pins it outputs on can't be used for DAC output without modding the m181 PCB
	// even so, the FPU alone would be a huge gain in speeding up the FP computation frame rate
	//
	// STM32F103CBT6 drop-in replacements .. STM32F303CBT6, STM32L412CBT6, STM32L431CCT6 and STM32L433CBT6

	#ifdef AVERAGE_PHASE
		if (!phi_table_ready)
		{	// create one time sample phase offset look-up table
			const float phi_step = (2 * M_PI) / (ADC_DATA_LENGTH * 2);     // 2 complete sine cycles per buffer
			for (unsigned int i = 0; i < ADC_DATA_LENGTH; i++)
			{
				const float phi = phi_step * i;
				phi_table[i] = t_complex(cosf(phi), sinf(phi));
			}
			phi_table_ready = 1;
		}
	#endif

	// tell DMA not to use the temp buffer
	tmp_buffer_in_use = 1;

	for (unsigned int buf_index = 0; buf_index < ARRAY_SIZE(adc_data); buf_index++)
	{
		const unsigned int vi_mode = buf_index >> 1;

		#if !defined(GOERTZEL_FILTER_LENGTH) || (GOERTZEL_FILTER_LENGTH <= 0)
			uint8_t filter = 0;	     // don't Goertzel filter
		#else
			uint8_t filter = 1;      // do Goertzel filter (a CPU with an FPU would speed this stage up no end)
		#endif

		uint8_t clipped = 0;

		if (op_mode == OP_MODE_MEASURING)
		{	// don't Goertzel filter if we're not going to use the data
			switch (vi_mode)
			{
				case VI_MODE_VOLT_LO_GAIN:
					if (!adc_data_clipped[VI_MODE_VOLT_HI_GAIN])
						clipped = 1;
					break;
				case VI_MODE_AMP_LO_GAIN:
					if (!adc_data_clipped[VI_MODE_AMP_HI_GAIN])
						clipped = 1;
					break;
				case VI_MODE_VOLT_HI_GAIN:
					if (adc_data_clipped[VI_MODE_VOLT_HI_GAIN])
						clipped = 1;
					break;
				case VI_MODE_AMP_HI_GAIN:
					if (adc_data_clipped[VI_MODE_AMP_HI_GAIN])
						clipped = 1;
					break;
			}
		}

		if (!filter || clipped)
		{	// don't Goertzel DFT filter the waveform, but do do these ..
			//   compute waveform RMS magnitude
			//   compute waveform phase

			// point to the ADC samples
			register float *buf = adc_data[buf_index];

			{	// compute waveform RMS magnitude of the unfiltered waveform
				register float sum = 0;
				for (unsigned int k = 0; k < ADC_DATA_LENGTH; k++)
					sum += SQR(buf[k]);
				sum *= 1.0f / ADC_DATA_LENGTH;
				mag_rms[buf_index] = sqrtf(sum);
			}

			{	// compute waveform phase using a single Goertzel DFT on the unfiltered waveform
				const t_complex g = goertzel_block(buf, ADC_DATA_LENGTH, &goertzel);
				phase_deg[buf_index] = (g.real != 0) ? fmodf((atan2f(g.imag, g.real) * RAD_TO_DEG) + 270, 360) : NAN;
			}
		}
		else
		{	// use many Goertzel DFT's to filter/clean the entire waveform
			// compute waveform RMS magnitude
			// compute waveform phase

			// point to an output buffer for the Goertzel filter to save into
			register t_complex *tmp_buf = (t_complex *)tmp_buffer;

			register float *buf = adc_data[buf_index];

			// filter the entire waveform using many Goertzel DFTs
			if (goertzel_wrap(buf, tmp_buf, ADC_DATA_LENGTH, GOERTZEL_FILTER_LENGTH, &goertzel) < 0)
				return -1;        // their was a key press whilst doing it, drop everthing (on this run) to immediately process the users input

			{	// compute RMS magnitude and save the Goertzel filtered output samples
				register float sum = 0;
				for (unsigned int k = 0; k < ADC_DATA_LENGTH; k++)
				{
					register const t_complex samp = tmp_buf[k];  // fetch filtered waveform sample
					sum += SQR(samp.real) + SQR(samp.imag);      // sum it (for computing the average)
					buf[k] = samp.real;                          // save the Goertzel filtered sample
				}
				sum *= 1.0f / ADC_DATA_LENGTH;
				mag_rms[buf_index] = sqrtf(sum);                 // save the computed RMS magnitude
			}

			#ifndef AVERAGE_PHASE
			{	// compute waveform phase using a single Goertzel DFT on the filtered waveform
				const t_complex s = goertzel_block(buf, ADC_DATA_LENGTH, &goertzel);
				phase_deg[buf_index] = (s.real != 0) ? fmodf((atan2f(s.imag, s.real) * RAD_TO_DEG) + 270, 360) : NAN;
			}
			#else
			{	// compute waveform phase using all the Goertzel DFTs that filtered the entire waveform
				// ie, compute the average phase
				register t_complex sum;
				for (unsigned int k = 0; k < ADC_DATA_LENGTH; k++)
				{
					// remove the phase offset from this sample position in the buffer
					register const t_complex p = phi_table[k];   // fetch sample phase in the buffer
					register       t_complex s = tmp_buf[k];     // fetch filtered waveform sample
					// conj multiply
					s = t_complex((s.real * p.real) + (s.imag * p.imag), (s.real * p.imag) - (s.imag * p.real));  // phase difference

					sum.real += s.real;
					sum.imag += s.imag;
				}
				phase_deg[buf_index] = (sum.real != 0) ? fmodf((atan2f(sum.imag, sum.real) * RAD_TO_DEG) + 270, 360) : NAN;
			}
			#endif
		}
	}

	// free up the temp buffer
	tmp_buffer_in_use = 0;

	return 0;
}

// combine AFC mag/phase results (the AFC sample blocks are all the same so may as well average them together)
//
void combine_afc(float *avg_rms, float *avg_deg)
{

	unsigned int sum_count = 0;
	float        sum_rms   = 0;
	t_complex       sum_phase = {0, 0};

	for (unsigned int mode = 0; mode < 8; mode += 2)
	{
		if (op_mode == OP_MODE_MEASURING)
		{
			// don't bother using this AFC if any clipping/saturation has been detected on this mode
			switch (mode >> 1)
			{
				case VI_MODE_VOLT_LO_GAIN:
					if (!adc_data_clipping[VI_MODE_VOLT_HI_GAIN])
						continue;
					break;
				case VI_MODE_AMP_LO_GAIN:
					if (!adc_data_clipping[VI_MODE_AMP_HI_GAIN])
						continue;
					break;
				case VI_MODE_VOLT_HI_GAIN:
					if (adc_data_clipping[VI_MODE_VOLT_HI_GAIN])
						continue;
					break;
				case VI_MODE_AMP_HI_GAIN:
					if (adc_data_clipping[VI_MODE_AMP_HI_GAIN])
						continue;
					break;
			}
		}

		sum_rms += mag_rms[mode + 1];

		const float phase_rad = phase_deg[mode + 1] * DEG_TO_RAD;
		sum_phase.real += cosf(phase_rad);
		sum_phase.imag += sinf(phase_rad);

		sum_count++;
	}

	*avg_rms = sum_rms / sum_count;
	*avg_deg = (sum_phase.real != 0) ? atan2f(sum_phase.imag, sum_phase.real) * RAD_TO_DEG : NAN;
}

#if defined(MEDIAN_SIZE) && (MEDIAN_SIZE >= 3)

	// qsort compare function
	//
	int compare_float(const void *a, const void *b)
	{
		return (*(float *)a - *(float *)b);
	}

	// median filter the magnitude and phase results to help reduce noise
	//
	void median_filter(void)
	{
		static int median_buffer_index = -1;

		static float mag_rms_median_buffer[ARRAY_SIZE(mag_rms)][MEDIAN_SIZE];
		static float phase_deg_median_buffer[ARRAY_SIZE(phase_deg)][MEDIAN_SIZE];

		if (!gain_changed && median_buffer_index >= 0)
		{	// median filters

			for (unsigned int m = 0; m < ARRAY_SIZE(mag_rms); m++)
			{
				float sort_buffer[MEDIAN_SIZE];

				{	// mag_rms median

					// add new value into the input buffer
					mag_rms_median_buffer[m][median_buffer_index] = mag_rms[m];

					// sort
					memcpy(sort_buffer, mag_rms_median_buffer[m], sizeof(sort_buffer));
					qsort(sort_buffer, MEDIAN_SIZE, sizeof(sort_buffer[0]), compare_float);

					// save the median
					mag_rms[m] = sort_buffer[MEDIAN_SIZE / 2];
				}

				{	// phase_deg median

					const float deg = phase_deg[m];

					// add new value into the input buffer
					phase_deg_median_buffer[m][median_buffer_index] = deg;

					// sort - has to take into account that angles wrap-a-round 0-360/360-0
					for (unsigned int i = 0; i < MEDIAN_SIZE; i++)
						sort_buffer[i] = phase_diff(deg, phase_deg_median_buffer[m][i]);
					qsort(sort_buffer, MEDIAN_SIZE, sizeof(sort_buffer[0]), compare_float);

					// save the median
					phase_deg[m] = deg + sort_buffer[MEDIAN_SIZE / 2];
				}

				// update buffer 'write' index
				if (++median_buffer_index >= (int)MEDIAN_SIZE)
					median_buffer_index = 0;
			}
		}
		else
		{	// reset input buffers
			median_buffer_index = 0;

			for (unsigned int m = 0; m < ARRAY_SIZE(mag_rms); m++)
			{
				const float mag = mag_rms[m];
				for (unsigned int i = 0; i < MEDIAN_SIZE; i++)
					mag_rms_median_buffer[m][i] = mag;
			}

			for (unsigned int m = 0; m < ARRAY_SIZE(phase_deg); m++)
			{
				const float deg = phase_deg[m];
				for (unsigned int i = 0; i < MEDIAN_SIZE; i++)
					phase_deg_median_buffer[m][i] = deg;
			}
		}
	}

#endif

// process the new averaged sample blocks to finally create the wanted final DUT parameters
//
void process_data(void)
{
	if (display_hold)
		return;

	if (process_Goertzel() < 0)
		return;

	#if 1
	{	// combine all used AFC results into one - good idea, or not ?

		float afc_rms;
		float afc_deg;

		combine_afc(&afc_rms, &afc_deg);

		mag_rms[1]   = afc_rms;
		mag_rms[3]   = afc_rms;
		mag_rms[5]   = afc_rms;
		mag_rms[7]   = afc_rms;

		phase_deg[1] = afc_deg;
		phase_deg[3] = afc_deg;
		phase_deg[5] = afc_deg;
		phase_deg[7] = afc_deg;
	}
	#endif

	#if defined(MEDIAN_SIZE) && (MEDIAN_SIZE >= 3)
		if (op_mode == OP_MODE_MEASURING)
			median_filter();                          // median filter to improve display reading stabilisation
	#endif

	// gain path decision
	// use the high gain samples only if they aren't clipping/saturating
	//
//	volt_gain_sel = adc_data_clipped[VI_MODE_VOLT_HI_GAIN] ? 0 : 1;  // '0' = LO gain mode   '1' = HI gain mode
//	amp_gain_sel  = adc_data_clipped[VI_MODE_AMP_HI_GAIN]  ? 0 : 1;  // '0' = LO gain mode   '1' = HI gain mode

	// waveform amplitudes
	//
	system_data.rms_voltage_adc = mag_rms[(volt_gain_sel * 4) + 0];
	system_data.rms_voltage_afc = mag_rms[(volt_gain_sel * 4) + 1];
	//
	system_data.rms_current_adc = mag_rms[(amp_gain_sel  * 4) + 2];
	system_data.rms_current_afc = mag_rms[(amp_gain_sel  * 4) + 3];

	system_data.rms_current_adc *= inv_series_ohms;
	system_data.rms_current_afc *= inv_series_ohms;

	// TODO: calibrate the 'high_gain' value by computing the actual gain from the calibration results

	const float high_scale = 1.0f / high_gain;
	const float v_scale    = volt_gain_sel ? high_scale : 1.0f;
	const float i_scale    = amp_gain_sel  ? high_scale : 1.0f;

	// scale according to which gain path is being used
	system_data.rms_voltage_adc *= v_scale;
	system_data.rms_voltage_afc *= v_scale;
	system_data.rms_current_adc *= i_scale;
	system_data.rms_current_afc *= i_scale;

	system_data.impedance = system_data.rms_voltage_adc / system_data.rms_current_adc;

	#if 1
		if (op_mode == OP_MODE_MEASURING)
		{
			const unsigned int freq_index = (measurement_Hz <= 300) ? 0 : 1;   // 100Hz/1kHz

			float zo = 0;
			float zs = 0;

			if (settings.open_probe_calibration->done)
			{	// apply open probe calibration
				const float v_cal_rms = settings.open_probe_calibration[freq_index].mag_rms[VI_MODE_VOLT_LO_GAIN * 2];
				const float i_cal_rms = settings.open_probe_calibration[freq_index].mag_rms[VI_MODE_AMP_HI_GAIN  * 2] * inv_series_ohms * high_scale;
			 	zo = v_cal_rms / i_cal_rms;
			}

			if (settings.shorted_probe_calibration->done)
			{	// apply shorted probe calibration
				const float v_cal_rms = settings.shorted_probe_calibration[freq_index].mag_rms[VI_MODE_VOLT_HI_GAIN * 2] * high_scale;
				const float i_cal_rms = settings.shorted_probe_calibration[freq_index].mag_rms[VI_MODE_AMP_LO_GAIN  * 2] * inv_series_ohms;
				zs = v_cal_rms / i_cal_rms;
			}

			//const float zstd = ;  // used with open/short/load calibration
			//const float zsm  = ;  // used with open/short/load calibration
			const float zxm = system_data.impedance;
			if (settings.open_probe_calibration->done && settings.shorted_probe_calibration->done)
				//system_data.impedance = zstd * (((zs - zxm) * (zsm - zo)) / ((zxm - zo) * (zs - zsm)));   // page 136 Keysight-Technologies-impedance-measurement-handbook.pdf
//				system_data.impedance = (zo * (zxm - zs)) / (zo - (zxm - zs));
				system_data.impedance = zo * ((zs - zxm) / (zxm - zo));   // page 135 Keysight-Technologies-impedance-measurement-handbook.pdf
			else
			if (settings.open_probe_calibration->done)
				system_data.impedance = (zo * zxm) / (zo - zxm);
			//else
			//if (settings.shorted_probe_calibration->done)

			system_data.impedance = fabsf(system_data.impedance);
		}
	#endif

	#if 0
		// sanity check
		if (op_mode == OP_MODE_MEASURING)
			system_data.impedance = (system_data.impedance > 10e9f) ? 10e9f : system_data.impedance;
	#endif

	system_data.voltage_phase_deg = phase_diff(phase_deg[(volt_gain_sel * 4) + 0], phase_deg[(volt_gain_sel * 4) + 1]);   // phase difference between ADC and AFC waves
	system_data.current_phase_deg = phase_diff(phase_deg[(amp_gain_sel  * 4) + 2], phase_deg[(amp_gain_sel  * 4) + 3]);   // phase difference between ADC and AFC waves
	//
	system_data.vi_phase_deg      = phase_diff(system_data.voltage_phase_deg, system_data.current_phase_deg);             // phase difference between voltage and current waves

	if (op_mode != OP_MODE_MEASURING)
		return;                          // doing the open/short calibration runs, no need to go any further here

	// **************************
	// compute the DUT (L, C or R) parameters using the above measurements

	const float omega            = (float)(2 * M_PI) * measurement_Hz;               // angular frequency (in rad/s) .. ω = 2 PI Hz

	const float vi_phase_rad     = system_data.vi_phase_deg * DEG_TO_RAD;
	const float cs               = cosf(vi_phase_rad);
	const float sn               = sinf(vi_phase_rad);

//	const float ser_resistive    = system_data.impedance * fabsf(cs);                // Rs
//	const float ser_reactance    = system_data.impedance * fabsf(sn);                // Xs
	const float ser_resistive    = system_data.impedance * cs;                       // Rs
	const float ser_reactance    = system_data.impedance * fabsf(sn);                // Xs

	if (ser_resistive == 0 || ser_reactance == 0 || omega == 0)
		return;                                                                      // prevent div-by-0 errors

	const float ser_inductance   = ser_reactance / omega;                            // L = X / ω
	const float ser_capacitance  = 1.0f / (omega * ser_reactance);                   // C = 1 / (ωX)
	const float ser_esr          = ser_resistive;                                    // R
	const float ser_tan_delta    = ser_resistive / ser_reactance;                    // D = R / X
//	const float ser_qf           = (omega * ser_inductance) / ser_resistive;         // Q = (ωL) / R
//	const float ser_qf           = 1.0f / (omega * ser_capacitance * ser_resistive); // Q = 1 / (ωCR)
	const float ser_qf           = ser_reactance / ser_resistive;                    // Q = X / R or 1 / D

	// series to parallel
	const float p                = SQR(ser_resistive) + SQR(ser_reactance);
	const float par_resistive    = p / ser_resistive;
	const float par_reactance    = p / ser_reactance;
	const float par_inductance   = par_reactance / omega;                            // L = X / ω
	const float par_capacitance  = 1.0f / (omega * par_reactance);                   // C = 1 / (ωX)
	const float par_esr          = par_resistive;                                    // R
	const float par_tan_delta    = par_resistive / par_reactance;                    // D = R / X
//	const float par_qf           = (omega * par_inductance) / par_resistive;         // Q = (ωL) / R
//	const float par_qf           = 1.0f / (omega * par_capacitance * par_resistive); // Q = 1 / (ωCR)
	const float par_qf           = par_reactance / par_resistive;                    // Q = X / R or 1 / D

	system_data.series.inductance    = ser_inductance;
	system_data.series.capacitance   = ser_capacitance;
	system_data.series.resistance    = ser_resistive; // system_data.impedance;
	system_data.series.esr           = ser_esr;
	system_data.series.tan_delta     = ser_tan_delta;
	system_data.series.qf            = ser_qf;
	system_data.series.reactance     = ser_reactance;

	system_data.parallel.inductance  = par_inductance;
	system_data.parallel.capacitance = par_capacitance;
	system_data.parallel.resistance  = par_resistive; // system_data.impedance;
	system_data.parallel.esr         = par_esr;
	system_data.parallel.tan_delta   = par_tan_delta;
	system_data.parallel.qf          = par_qf;
	system_data.parallel.reactance   = par_reactance;

	// **************************
	// sanity checks

	#if 1
		if (op_mode == OP_MODE_MEASURING)
		{
			system_data.series.inductance    = (system_data.series.inductance  >    1e3f) ? 1e3f  : system_data.series.inductance;
			system_data.series.capacitance   = (system_data.series.capacitance <      0) ? 0      : system_data.series.capacitance;
			system_data.series.resistance    = (system_data.series.resistance  > 100e6f) ? 100e6f : system_data.series.resistance;

			system_data.parallel.inductance  = (system_data.parallel.inductance  >   1e3f) ? 1e3f   : system_data.parallel.inductance;
			system_data.parallel.capacitance = (system_data.parallel.capacitance <      0) ? 0      : system_data.parallel.capacitance;
			system_data.parallel.resistance  = (system_data.parallel.resistance  > 100e6f) ? 100e6f : system_data.parallel.resistance;
		}
	#endif
}

// save the new ADC DMA sample block
//
// this is called from the ADC DMA interrupt
//
void process_ADC_DMA(const void *buffer)
{
	if (initialising || tmp_buffer_in_use)
		return;               // the temp buffer is not available for use, drop this sample block

	if (vi_measure_index >= VI_MODE_DONE)
		return;               // wait till the exec has done it's thing

	LL_GPIO_SetOutputPin(LED_GPIO_Port, LED_Pin);                          // TEST only, LED on

	// copy the new ADC sample block as quickly as possible so as not to hold up the DMA
	memcpy(tmp_buffer, buffer, sizeof(t_adc_dma_data_16) * ADC_DATA_LENGTH);
	tmp_buffer_in_use = 1;    // let the exec know that their is a new sample block ready
}

// process the new ADC raw sample block that the ADSC DMA saved for us
//
// this is called from the main exec loop
//
void process_ADC_exec(void)
{
	// process the new ADC 12-bit samples

	if (!tmp_buffer_in_use)
		return;               // DMA hasn't yet given us a new sample block

	// point to the new ADC sample block from the DMA
	const t_adc_dma_data_16 *adc_buffer = (t_adc_dma_data_16 *)tmp_buffer;

	// current VI mode index
	unsigned int vi_index = vi_measure_index;

	// ignore this sample block if we've completed the VI measurement scan
	if (vi_index >= VI_MODE_DONE)
	{
		tmp_buffer_in_use = 0;
		return;
	}

	// use a table to set HD mode pins in a custom order to cope with a HW design floor
	const unsigned int vi_mode = vi_measure_mode_table[vi_index];

	if (vi_index == 0 && adc_buffer_sum_count == 0)
	{	// the first sample block after setting the HW mode pins

		set_measure_mode_pins(vi_mode);                                     // ensure the HW mode pins are set correctly

		// reset the waveform clip/saturation detection flags (one for each VI mode)
		memset(adc_data_clipping, 0, sizeof(adc_data_clipping));

//		LL_GPIO_SetOutputPin(LED_GPIO_Port, LED_Pin);                       // TEST only, LED on
	}

	// each time the HW VI mode is changed, the ADC input sees a large unwanted spike/DC-offset that takes time to settle :(
	// so we simply discard a number of sample blocks after each HW VI mode change

	// skip less blocks if the gain pin hasn't changed
//	const unsigned int skip_block_count = ((vi_mode >> 1) == (prev_vi_mode >> 1)) ? MODE_SWITCH_BLOCK_WAIT_SHORT : MODE_SWITCH_BLOCK_WAIT_LONG;
	const unsigned int skip_block_count = MODE_SWITCH_BLOCK_WAIT_LONG;

	if (adc_buffer_sum_count >= skip_block_count)
	{	// add the new sample block to the averaging buffer (to reduce noise)

		// invert the current (I) ADC waveform to counter-act the inverting OP-AMP stage
		register const int16_t adc_sign = (vi_mode == VI_MODE_AMP_LO_GAIN || vi_mode == VI_MODE_AMP_HI_GAIN) ? -1 : 1;
		register const int16_t afc_sign = 1;

		if (vi_mode >= VI_MODE_VOLT_HI_GAIN)
		{	// only bother doing a histogram for the HIGH gain modes, the LOW gain modes never clip/saturate

			// for detecting waveform clipping
			#define HISTOGRAM_SIZE        (2048 / (1 << 5))
			uint8_t histogram[HISTOGRAM_SIZE + 1] = {0};               // '+ 1' so we don't try writing beyond the buffer size

			const uint8_t threshold = ADC_DATA_LENGTH / 14;            // histogram spike threshold level

			for (unsigned int i = 0; i < ADC_DATA_LENGTH; i++)
			{
				// fetch the raw ADC samples
				register const int16_t adc = (adc_buffer[i].adc - 2048) * adc_sign;
				register const int16_t afc = (adc_buffer[i].afc - 2048) * afc_sign;

				// add the new samples to the averaging buffer
				adc_buffer_sum[i].adc += adc;
				adc_buffer_sum[i].afc += afc;

				// update the histogram with the sample
				//register uint32_t val = (adc < 0) ? -adc : adc;      // 0..2048   two's compliment
				register uint32_t val = (adc < 0) ? ~adc : adc;        // 0..2047   one's compliment
				val >>= 5;                                             // 0 to HISTOGRAM_SIZE
				histogram[val]++;                                      // increment the histogram bin
			}

			// check to see any clipping/saturation is occuring
			// we do this by looking for any spikes in the upper section of the histogram
			register uint8_t clipped = 0;
			register uint8_t p0      = 0;
			register uint8_t p1      = 0;
			for (unsigned int i = HISTOGRAM_SIZE * 0.7; i < ARRAY_SIZE(histogram) && !clipped; i++)
			{
				#if 0
					// look at the absolute histogram level (rather than spikes)
					if (histogram[i] >= threshold)
						clipped = 1;
				#else
					// look for histogram spike
					register const uint8_t p2         = histogram[i];
					register const uint8_t lead_edge  = (p1 >= p0) ? p1 - p0 : 0;
					register const uint8_t trail_edge = (p1 >= p2) ? p1 - p2 : 0;
					p0 = p1;
					p1 = p2;
					//clipped = (lead_edge >= threshold && trail_edge >= threshold) ? 1 : clipped; // spikes leading and trailing edge
					clipped = (lead_edge >= threshold || trail_edge >= threshold) ? 1 : clipped;   // spikes leading or trailing edge
					//clipped = (lead_edge >= threshold) ? 1 : clipped;                            // spikes leading edge only
					//clipped = (trail_edge >= threshold) ? 1 : clipped;                           // spikes trailing edge only
				#endif
			}
			adc_data_clipping[vi_mode] |= clipped;                // '1' = detected clipped/saturated samples
		}
		else
		{	// forget doing a histogram in the LOW gain modes as the ADC samples never clip/saturate, so no need to check for that
			for (unsigned int i = 0; i < ADC_DATA_LENGTH; i++)
			{
				adc_buffer_sum[i].adc += (adc_buffer[i].adc - 2048) * adc_sign;
				adc_buffer_sum[i].afc += (adc_buffer[i].afc - 2048) * afc_sign;
			}
			adc_data_clipping[vi_mode] = 0;                       // '0' = no clipping/saturation detected
		}
	}

	// free up the temporary buffer
	tmp_buffer_in_use = 0;

	// decide which modes are useful (or not)
	//
	// we drop the hi-gain blocks if they are clipped (makes the data useless)
	// we drop the lo-gain blocks if the hi-gain blocks are usable (no high-gain clipping detected)
	//
	unsigned int average_count = (128ul * measurement_Hz) >> 10;  // the higher the measurement Hz the more buffers we average
//	unsigned int average_count = SLOW_ADC_AVERAGE_COUNT;
	if (op_mode == OP_MODE_MEASURING)
	{
		if (display_hold)
			average_count = (8ul * measurement_Hz) >> 10;  // the higher the measurement Hz the more buffers we average
//			average_count = 8;                           // display is paused, we're doing nothing but sending the samples to the PC (average any number of blocks you want here)
		else
		if (adc_data_clipping[vi_mode])
			average_count = 1;                           // this block of samples are clipping, drop them, move on to the next mode
		else
		if (vi_mode < VI_MODE_VOLT_HI_GAIN && !adc_data_clipped[VI_MODE_VOLT_HI_GAIN + vi_mode])
			average_count = 1;                           // hi-gain samples were not clipped on the previous run, so drop these lo-gain samples
		else
		if (settings.flags & SETTING_FLAG_FAST_UPDATES)
			average_count = (16ul * measurement_Hz) >> 10;  // the higher the measurement Hz the more buffers we average
//			average_count = FAST_ADC_AVERAGE_COUNT;      // user has selected faster display updates, which just means we average less blocks together
	}
	average_count = (average_count < 1) ? 1 : average_count;

	LL_GPIO_ResetOutputPin(LED_GPIO_Port, LED_Pin);                          // TEST only, LED off

	if (++adc_buffer_sum_count < (skip_block_count + average_count))
	{	// not yet summed the desired number of sample blocks
		return;
	}




	// we've now summed the desired number of sample blocks, so are ready to be re-scaled, saved and DC offset removed ..

	// get ready for next measurement VI mode
	vi_index++;

	// set the GS/VI pins ready for the next measurement run
	set_measure_mode_pins(vi_measure_mode_table[vi_index]);

	{	// fetch & re-scale the summed ADC sample blocks to create an averaged single block (reduces noise)

		const unsigned int buf_index = vi_mode * 2;

		register float *buf_adc = adc_data[buf_index + 0];
		register float *buf_afc = adc_data[buf_index + 1];

		{	// fetch and re-scale (also converts to 'float' type at the same time)
			register const float scale = 1.0f / (adc_buffer_sum_count - skip_block_count);
			for (unsigned int i = 0; i < ADC_DATA_LENGTH; i++)
			{
				buf_adc[i] = adc_buffer_sum[i].adc * scale;
				buf_afc[i] = adc_buffer_sum[i].afc * scale;
			}
		}

		{
			// remove any DC offset (there's always some) from this new averaged block of samples

			const float coeff = (frames <= 3) ? 0.9 : 0.3;  // fast LPF covergence to start with, then switch to slower coeff

			if (!adc_data_clipping[vi_mode] || op_mode != OP_MODE_MEASURING)   // don't bother if the samples are clipped (makes the block useless)
			{	// ADC input

				// compute the DC offset
				register float sum = 0;
				for (unsigned int i = 0; i < ADC_DATA_LENGTH; i++)
					sum += buf_adc[i];
				sum *= 1.0f / ADC_DATA_LENGTH;

				// LPF
				if (!adc_data_clipping[vi_mode])
					settings.input_offset.adc[vi_mode] = ((1.0f - coeff) * settings.input_offset.adc[vi_mode]) + (coeff * sum);

				if (!display_hold)                            // don't bother remove offset if display HOLD is active
				{	// subtract/remove the DC offset
					if (!adc_data_clipping[vi_mode])
						sum = settings.input_offset.adc[vi_mode];
					for (unsigned int i = 0; i < ADC_DATA_LENGTH; i++)
						buf_adc[i] -= sum;
				}
			}

			{	// AFC input (samples never ever clip)

				// compute the DC offset
				register float sum = 0;
				for (unsigned int i = 0; i < ADC_DATA_LENGTH; i++)
					sum += buf_afc[i];
				sum *= 1.0f / ADC_DATA_LENGTH;

				// LPF
				settings.input_offset.afc[vi_mode] = ((1.0f - coeff) * settings.input_offset.afc[vi_mode]) + (coeff * sum);

				if (!display_hold)                            // don't remove offset if display HOLD is active
				{	// subtract/remove the DC offset
					sum = settings.input_offset.afc[vi_mode];
					for (unsigned int i = 0; i < ADC_DATA_LENGTH; i++)
						buf_afc[i] -= sum;
				}
			}

		}
	}

	// reset averaging buffer ready for the next measurement run
	memset(adc_buffer_sum, 0, sizeof(adc_buffer_sum));
	adc_buffer_sum_count = 0;

	// remember current VI mode
	prev_vi_mode = vi_mode;

	// save the new VI mode for the next measurement run
	vi_measure_index = vi_index;

	if (vi_index >= VI_MODE_DONE)
	{
		gain_changed = (memcmp(adc_data_clipped, adc_data_clipping, sizeof(adc_data_clipped)) != 0) ? 1 : 0;    // '1' if gain selection changed

		memcpy(adc_data_clipped, adc_data_clipping, sizeof(adc_data_clipped));   // save the new clip detection flags
		memset(adc_data_clipping, 0, sizeof(adc_data_clipping));                 // reset ready for next run

//		if (!display_hold && gain_changed)
		if (!display_hold)
		{
			// gain path decision
			// use the high gain samples only if they aren't clipping/saturating
			//
			volt_gain_sel = adc_data_clipped[VI_MODE_VOLT_HI_GAIN] ? 0 : 1;      // '0' = LOW gain mode   '1' = HIGH gain mode
			amp_gain_sel  = adc_data_clipped[VI_MODE_AMP_HI_GAIN]  ? 0 : 1;      // '0' = LOW gain mode   '1' = HIGH gain mode
		}
	}
}

// ***********************************************************

// available font sizes ..
//
//  Font_7x10  .. small
//  Font_11x18 .. bigger than small
//  Font_16x26 .. bigger than the bigger than small

//#define DRAW_LINES          // if you want horizontal lines drawn
//#define DISPLAY_LCR_MODE    // if you want the 'R' 'L' or 'C' before the DUT value

#define LINE_SPACING     13

#ifdef DRAW_LINES
	#define LINE2_Y      (LINE_SPACING + 5)
	#define LINE3_Y      (10 + LINE2_Y - 1 + LINE_SPACING)
#else
	#define LINE2_Y      (LINE_SPACING + 3)
	#define LINE3_Y      (10 + LINE2_Y + 1 + LINE_SPACING)
#endif

void print_sprint(const unsigned int digit, const float value, char output_char[], const unsigned int out_max_size)
{
//	float val = value;
//	const char unit = unit_conversion(&val);

	const float v = fabsf(value);

	switch (digit)
	{
		case 2:
	        if (v < 10)
    	        snprintf(output_char, out_max_size, "%0.1f", value); // 1.2
        	else
            	snprintf(output_char, out_max_size, "%0.0f", value); // 12 (no dp)
			break;

		case 3:
	        if (v < 10)
				snprintf(output_char, out_max_size, "%0.2f", value); // 1.23
        	else
			if (v < 100)
				snprintf(output_char, out_max_size, "%0.1f", value); // 12.3
	        else
				snprintf(output_char, out_max_size, "%0.0f", value); // 123 (no dp)
			break;

		default:
		case 4:
	        if (v < 10)
				snprintf(output_char, out_max_size, "%0.3f", value); // 1.234
        	else
			if (v < 100)
				snprintf(output_char, out_max_size, "%0.2f", value); // 12.34
			else
			if (v < 1000)
				snprintf(output_char, out_max_size, "%0.1f", value); // 123.4
	        else
				snprintf(output_char, out_max_size, "%0.0f", value); // 1234 (no dp)
			break;
    }
}

void print_custom_symbol(const unsigned int startX, const unsigned int startY, const uint16_t symbol[], const unsigned int symbolWidth, const unsigned int symbolHeight)
{
    // For each row of the symbol...
    for (unsigned int row = 0; row < symbolHeight; row++)
    {
        // Shift the row data left to align active pixels in the MSB position.
        // (For a symbolWidth of 7, shift left by 16 - 7 = 9 bits.)
        uint16_t rowData = symbol[row] << (16u - symbolWidth);

        // For each column in this row...
        for (unsigned int col = 0; col < symbolWidth; col++)
        {
            // if MSB set, draw a white pixel
			ssd1306_DrawPixel(startX + col, startY + row, (rowData & 0x8000) ? White : Black);
            rowData <<= 1;
        }
    }
}

void screen_init(void)
{
	I2C_bus_init();

	ssd1306_Init();

	// flash up WHITE to highlight possible bad pixels
	ssd1306_Fill(White);
	ssd1306_UpdateScreen();

	LL_mDelay(500);
}

void bootup_screen(void)
{
	ssd1306_Fill(Black);

	ssd1306_SetCursor(0, 0);
	ssd1306_WriteString("M181", &Font_7x10, White);

	sprintf(str_buf, "%lu", settings.baudrate);
	ssd1306_SetCursor(5 * Font_7x10.width, 0);
	ssd1306_WriteString(str_buf, &Font_7x10, White);

	sprintf(str_buf, "v%.2f", FW_VERSION);
	ssd1306_SetCursor(SSD1306_WIDTH - 1 - (strlen(str_buf) * Font_7x10.width), 0);
	ssd1306_WriteString(str_buf, &Font_7x10, White);

	ssd1306_SetCursor(16, 14);
	ssd1306_WriteString("LCR Meter", &Font_11x18, White);

	// dotted line
	ssd1306_dotted_hline(0, SSD1306_WIDTH - 1, 3, 32 - 1, White);

	ssd1306_SetCursor(5, 38);
	ssd1306_WriteString("HW by JYETech", &Font_7x10, White);
	ssd1306_SetCursor(5, 52);
	ssd1306_WriteString("FW by Jai & 1o11", &Font_7x10, White);

	ssd1306_UpdateScreen();
}

void draw_measurement_mode(void)
{
	ssd1306_SetCursor(SSD1306_WIDTH - 1 - (1 * Font_7x10.width), 0);
	snprintf(str_buf, sizeof(str_buf), "%u", system_data.vi_measure_mode);
	ssd1306_WriteString(str_buf, &Font_7x10, White);

	ssd1306_UpdateScreen();
}

void draw_screen(void)
{
	const uint8_t x1 = 20;
	const uint8_t x2 = 62;
//	const uint8_t x3 = 75;

	const uint8_t par = settings.flags & SETTING_FLAG_PARALLEL;

	// clear the screen
	ssd1306_Fill(Black);

	switch (op_mode)
	{
		default:
		case OP_MODE_MEASURING:

			// ***************************
			// Line 1

			// serial/parallel mode
			ssd1306_SetCursor(0, 0);
			ssd1306_WriteString(par ? "par " : "ser ", &Font_7x10, White);

			// measurement frequency
			if (measurement_Hz < 1000)
				snprintf(str_buf, sizeof(str_buf), "%u", measurement_Hz);
			else
				snprintf(str_buf, sizeof(str_buf), "%0.3fk", measurement_Hz * 1e-3f);
			trim_trailing_zeros(str_buf);
			//ssd1306_MoveCursor(3, 0);
			ssd1306_WriteString(str_buf, &Font_7x10, White);

			{	// open/short calibration
				const unsigned int index = (measurement_Hz <= 300) ? 0 : 1;
				unsigned int i = 0;
				memset(str_buf, 0, sizeof(str_buf));
				str_buf[i++] = settings.open_probe_calibration[index].done    ? 'O' : '-';
				str_buf[i++] = settings.shorted_probe_calibration[index].done ? 'S' : '-';
				ssd1306_SetCursor(11 * Font_7x10.width, 0);
				ssd1306_WriteString(str_buf, &Font_7x10, White);
			}

			// hold or fast
			ssd1306_SetCursor(SSD1306_WIDTH - 1 - (4 * Font_7x10.width), 0);
			if (display_hold)
				ssd1306_WriteString("HOLD", &Font_7x10, White);
			else
			if (settings.flags & SETTING_FLAG_FAST_UPDATES)
				ssd1306_WriteString("fast", &Font_7x10, White);
			#if 1
				else
				{	// VI phase
					print_sprint(4, system_data.vi_phase_deg, str_buf, sizeof(str_buf));
					ssd1306_SetCursor(SSD1306_WIDTH - 1 - (5 * Font_7x10.width), 0);
					ssd1306_WriteString(str_buf, &Font_7x10, White);
				}
			#endif

			// ***************************
			// horizontal line(s)

			#ifdef DRAW_LINES
				#if 0
					// solid line
					ssd1306_FillRectangle(0, LINE_SPACING - 1, SSD1306_WIDTH, LINE_SPACING, White);
				#else
					// dotted line
					ssd1306_dotted_hline(0, SSD1306_WIDTH, 3, LINE_SPACING - 1, White);
				#endif
			#endif

			// ***************************
			// Line 2

			{	// LCR mode

				float value = 0;

				switch (settings.lcr_mode)
				{
					case LCR_MODE_INDUCTANCE:
						value = par ? system_data.parallel.inductance : system_data.series.inductance;
						#if DISPLAY_LCR_MODE
							snprintf(str_buf, sizeof(str_buf), "L");
						#endif
						break;

					case LCR_MODE_CAPACITANCE:
						value = par ? system_data.parallel.capacitance : system_data.series.capacitance;
						#if DISPLAY_LCR_MODE
							snprintf(str_buf, sizeof(str_buf), "C");
						#endif
						break;

					case LCR_MODE_RESISTANCE:
						value = par ? system_data.parallel.resistance : system_data.series.resistance;
						#if DISPLAY_LCR_MODE
							snprintf(str_buf, sizeof(str_buf), "R");
						#endif
						break;

					case LCR_MODE_AUTO:
						// TODO:
						break;
				}

				ssd1306_SetCursor(0, LINE2_Y + 6);

				#if DISPLAY_LCR_MODE
					ssd1306_WriteString(str_buf, &Font_11x18, White);
				#endif

				char unit = unit_conversion(&value);

				switch (settings.lcr_mode)
				{
					case LCR_MODE_INDUCTANCE:
						if (unit == 'p')
						{
							unit = 'u';
							value *= 1e-6f;
						}
						else
						if (unit == 'n')
						{
							unit = 'u';
							value *= 1e-3f;
						}

						if (unit == 'u')
							snprintf(str_buf, sizeof(str_buf), "%0.1f", value);
						else
							print_sprint(4, value, str_buf, sizeof(str_buf));
						break;

					case LCR_MODE_CAPACITANCE:
						if (unit == 'p')
							snprintf(str_buf, sizeof(str_buf), "%0.1f", value);
						else
							print_sprint(4, value, str_buf, sizeof(str_buf));
						break;

					case LCR_MODE_RESISTANCE:
						if (unit == 'p')
						{
							unit = 'm';
							value *= 1e-9f;
						}
						else
						if (unit == 'n')
						{
							unit = 'm';
							value *= 1e-6f;
						}
						else
						if (unit == 'u')
						{
							unit = 'm';
							value *= 1e-3f;
						}

						if (unit == 'm')
							snprintf(str_buf, sizeof(str_buf), "%0.1f", value);
							//snprintf(str_buf, sizeof(str_buf), "%d", (int)value);
						else
							print_sprint(4, value, str_buf, sizeof(str_buf));
						break;

					case LCR_MODE_AUTO:
						break;
				}

				trim_trailing_zeros(str_buf);

				ssd1306_MoveCursor(4, -3);
				ssd1306_WriteString(str_buf, &Font_16x26, White);

				ssd1306_MoveCursor(6, -1);

				switch (settings.lcr_mode)
				{
					case LCR_MODE_INDUCTANCE:
					{
						unsigned int i = 0;
						if (unit != ' ')
							str_buf[i++] = unit;
						str_buf[i++] = 'H';
						str_buf[i++] = '\0';
						ssd1306_WriteString(str_buf, &Font_11x18, White);
						break;
					}

					case LCR_MODE_CAPACITANCE:
					{
						unsigned int i = 0;
						if (unit != ' ')
							str_buf[i++] = unit;
						str_buf[i++] = 'F';
						str_buf[i++] = '\0';
						ssd1306_WriteString(str_buf, &Font_11x18, White);
						break;
					}

					case LCR_MODE_RESISTANCE:
					{
						if (unit != ' ')
						{
							str_buf[0] = unit;
							str_buf[1] = '\0';
							ssd1306_WriteString(str_buf, &Font_11x18, White);
						}

						ssd1306_MoveCursor(3, 0);

						uint16_t x;
						uint16_t y;
						ssd1306_GetCursor(&x, &y);
						print_custom_symbol(x, y, omega_13x18, 13, 18);

						break;
					}

					case LCR_MODE_AUTO:
						break;
				}
			}

			#if 1
			{	// show gain setting for V and I modes
				str_buf[0] = volt_gain_sel ? 'H' : 'L';
				str_buf[1] = amp_gain_sel  ? 'H' : 'L';
				str_buf[2] ='\0';
				ssd1306_SetCursor(SSD1306_WIDTH - 1 - (2 * Font_7x10.width), LINE3_Y);
				ssd1306_WriteString(str_buf, &Font_7x10, White);
			}
			#endif

			// ***************************
			// Line 3

			# if 0
				{	// voltage
					float value = (system_data.rms_voltage_adc >= 0) ? system_data.rms_voltage_adc : 0;
					value = adc_to_volts(value);
					const char unit = unit_conversion(&value);

					ssd1306_SetCursor(0, LINE3_Y);
					print_sprint(4, value, str_buf, sizeof(str_buf));
					unsigned int i = strlen(str_buf);
					if (unit != ' ')
						str_buf[i++] = unit;
					str_buf[i++] = 'V';
					str_buf[i++] = '\0';
					trim_trailing_zeros(str_buf);
					ssd1306_WriteString(str_buf, &Font_7x10, White);
				}

				{	// current
					float value = (system_data.rms_current_adc >= 0) ? system_data.rms_current_adc : 0;
					value = adc_to_volts(value);
					const char unit = unit_conversion(&value);

					ssd1306_SetCursor(62, LINE3_Y);
					print_sprint(4, value, str_buf, sizeof(str_buf));
					unsigned int i = strlen(str_buf);
					if (unit != ' ')
						str_buf[i++] = unit;
					str_buf[i++] = 'A';
					str_buf[i++] = '\0';
					trim_trailing_zeros(str_buf);
					ssd1306_WriteString(str_buf, &Font_7x10, White);
				}
			#endif

			// ***************************
			// Bottom line

			switch (settings.lcr_mode)
			{
				case LCR_MODE_INDUCTANCE:
				case LCR_MODE_CAPACITANCE:

					{	// ESR

						float value = par ? system_data.parallel.esr : system_data.series.esr;
						const char unit = unit_conversion(&value);

						ssd1306_SetCursor(0, SSD1306_HEIGHT - 1 - Font_7x10.height);
						ssd1306_WriteString("ER", &Font_7x10, White);

						ssd1306_SetCursor(x1, SSD1306_HEIGHT - 1 - Font_7x10.height);

						print_sprint(3, value, str_buf, sizeof(str_buf));
						unsigned int i = strlen(str_buf);
						str_buf[i++] = unit;
						str_buf[i++] = '\0';
						trim_trailing_zeros(str_buf);
						ssd1306_WriteString(str_buf, &Font_7x10, White);
					}

					#if 0
					{	// Tan Delta

						float value = par ? system_data.parallel.tan_delta : system_data.series.tan_delta;
						const char unit = unit_conversion(&value);

						ssd1306_SetCursor(x2, SSD1306_HEIGHT - 1 - Font_7x10.height);
						ssd1306_WriteString("D", &Font_7x10, White);

						ssd1306_SetCursor(x3, SSD1306_HEIGHT - 1 - Font_7x10.height);

						print_sprint(3, value, str_buf, sizeof(str_buf));
						unsigned int i = strlen(str_buf);
						str_buf[i++] = unit;
						str_buf[i++] = '\0';
						trim_trailing_zeros(str_buf);
						ssd1306_WriteString(str_buf, &Font_7x10, White);
					}
					#else
					{	// Quality factor

						float value = par ? system_data.parallel.qf : system_data.series.qf;
						const char unit = unit_conversion(&value);

						ssd1306_SetCursor(x2, SSD1306_HEIGHT - 1 - Font_7x10.height);
						ssd1306_WriteString("Q ", &Font_7x10, White);

						//ssd1306_SetCursor(x3, SSD1306_HEIGHT - 1 - Font_7x10.height);
						print_sprint(3, value, str_buf, sizeof(str_buf));
						unsigned int i = strlen(str_buf);
						str_buf[i++] = unit;
						str_buf[i++] = '\0';
						trim_trailing_zeros(str_buf);
						ssd1306_WriteString(str_buf, &Font_7x10, White);
					}
					#endif

					break;

				case LCR_MODE_RESISTANCE:

					{	// inductance

						float value = par ? system_data.parallel.inductance : system_data.series.inductance;
						const char unit = unit_conversion(&value);

						ssd1306_SetCursor(0, SSD1306_HEIGHT - 1 - Font_7x10.height);

						print_sprint(4, value, str_buf, sizeof(str_buf));
						unsigned int i = strlen(str_buf);
						if (unit != ' ')
							str_buf[i++] = unit;
						str_buf[i++] = 'H';
						str_buf[i++] = '\0';
						trim_trailing_zeros(str_buf);
						ssd1306_WriteString(str_buf, &Font_7x10, White);
					}

					{	// Quality factor

						float value = par ? system_data.parallel.qf : system_data.series.qf;
						const char unit = unit_conversion(&value);

						ssd1306_SetCursor(x2, SSD1306_HEIGHT - 1 - Font_7x10.height);
						ssd1306_WriteString("Q ", &Font_7x10, White);

						print_sprint(3, value, str_buf, sizeof(str_buf));
						unsigned int i = strlen(str_buf);
						str_buf[i++] = unit;
						str_buf[i++] = '\0';
						trim_trailing_zeros(str_buf);
						ssd1306_WriteString(str_buf, &Font_7x10, White);
					}

					break;

				case LCR_MODE_AUTO:
					break;
			}

			// ***************************

			break;

		case OP_MODE_OPEN_PROBE_CALIBRATION:
			snprintf(str_buf, sizeof(str_buf), "OPEN cal %d", CALIBRATE_COUNT - calibrate.count - 1);
			ssd1306_SetCursor(0, 5);
			ssd1306_WriteString(str_buf, &Font_11x18, White);

			if (measurement_Hz < 1000)
				snprintf(str_buf, sizeof(str_buf), " %u Hz", measurement_Hz);
			else
				snprintf(str_buf, sizeof(str_buf), " %0.3f kHz", measurement_Hz * 1e-3f);
			trim_trailing_zeros(str_buf);
			ssd1306_SetCursor(0, LINE3_Y - 5);
			ssd1306_WriteString(str_buf, &Font_11x18, White);

			break;

		case OP_MODE_SHORTED_PROBE_CALIBRATION:
			snprintf(str_buf, sizeof(str_buf), "SHORT cal %d", CALIBRATE_COUNT - calibrate.count - 1);
			ssd1306_SetCursor(0, 5);
			ssd1306_WriteString(str_buf, &Font_11x18, White);

			if (measurement_Hz < 1000)
				snprintf(str_buf, sizeof(str_buf), " %u Hz", measurement_Hz);
			else
				snprintf(str_buf, sizeof(str_buf), " %0.3f kHz", measurement_Hz * 1e-3f);
			trim_trailing_zeros(str_buf);
			ssd1306_SetCursor(0, LINE3_Y - 5);
			ssd1306_WriteString(str_buf, &Font_11x18, White);

			break;
	}

	// ***************************
	// Bottom line: UART mode

	if (settings.flags & SETTING_FLAG_UART_DSO)
	{
		ssd1306_SetCursor(SSD1306_WIDTH - 1 - (1 * Font_7x10.width), SSD1306_HEIGHT - 1 - Font_7x10.height);
		ssd1306_WriteString((settings.flags & SETTING_FLAG_SEND_BINARY) ? "B" : "A", &Font_7x10, White);
	}

	// ***************************
	// write the display buffer to the screen

	ssd1306_UpdateScreen();

	draw_screen_count++;
}

// *************************************************************

void HAL_MspInit(void)
{
//	__HAL_RCC_AFIO_CLK_ENABLE();
//	__HAL_RCC_PWR_CLK_ENABLE();

//	__HAL_AFIO_REMAP_SWJ_NOJTAG();
}

void SystemClock_Config(void)
{
	LL_FLASH_SetLatency(LL_FLASH_LATENCY_2);
	while (LL_FLASH_GetLatency() != LL_FLASH_LATENCY_2) {}

	LL_RCC_HSE_Enable();
	while (LL_RCC_HSE_IsReady() != 1) {}

	LL_RCC_LSI_Enable();
	while (LL_RCC_LSI_IsReady() != 1) {}

	// 8MHz xtal, 72MHz system clock
	LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSE_DIV_1, LL_RCC_PLL_MUL_9);
	LL_RCC_PLL_Enable();
	while (LL_RCC_PLL_IsReady() != 1) {}

	LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
	LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_2);
	LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_1);

	LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);
	while (LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL) {}

	LL_SetSystemCoreClock(RCC_MAX_FREQUENCY);

	LL_RCC_SetADCClockSource(LL_RCC_ADC_CLKSRC_PCLK2_DIV_6);

//	if (HAL_InitTick(TICK_INT_PRIORITY) != HAL_OK)
//		Error_Handler();

	// enable the LL 1ms sys-tick
	HAL_SuspendTick();
	//LL_Init1msTick(RCC_MAX_FREQUENCY);
	LL_Init1msTick(SystemCoreClock);
	NVIC_SetPriority(SysTick_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), TICK_INT_PRIORITY, 0));
	LL_SYSTICK_EnableIT();
}

void MX_ADC_Init(void)
{
	LL_ADC_InitTypeDef       ADC_InitStruct       = {0};
	LL_ADC_CommonInitTypeDef ADC_CommonInitStruct = {0};
	LL_ADC_REG_InitTypeDef   ADC_REG_InitStruct   = {0};
//	LL_GPIO_InitTypeDef      GPIO_InitStruct      = {0};

	LL_TIM_DisableCounter(TIM3);

	LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOA);
	LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_ADC1);

	LL_GPIO_SetPinMode(ADC1_GPIO_Port, ADC1_Pin, LL_GPIO_MODE_ANALOG);
	LL_GPIO_SetPinMode(ADC2_GPIO_Port, ADC2_Pin, LL_GPIO_MODE_ANALOG);

	#ifdef DUAL_ADC_MODE

		// *********************************
		// ADC1

		LL_ADC_Disable(ADC1);
		LL_ADC_Disable(ADC2);

		{	// setup the ADC DMA

			LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA1);

			LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_1);

			LL_DMA_SetDataTransferDirection(DMA1, LL_DMA_CHANNEL_1, LL_DMA_DIRECTION_PERIPH_TO_MEMORY);
			LL_DMA_SetChannelPriorityLevel( DMA1, LL_DMA_CHANNEL_1, LL_DMA_PRIORITY_HIGH);
			LL_DMA_SetMode(                 DMA1, LL_DMA_CHANNEL_1, LL_DMA_MODE_CIRCULAR);
			LL_DMA_SetPeriphIncMode(        DMA1, LL_DMA_CHANNEL_1, LL_DMA_PERIPH_NOINCREMENT);
			LL_DMA_SetMemoryIncMode(        DMA1, LL_DMA_CHANNEL_1, LL_DMA_MEMORY_INCREMENT);
			LL_DMA_SetPeriphSize(           DMA1, LL_DMA_CHANNEL_1, LL_DMA_PDATAALIGN_WORD);
			LL_DMA_SetMemorySize(           DMA1, LL_DMA_CHANNEL_1, LL_DMA_MDATAALIGN_WORD);

			LL_DMA_ConfigAddresses(DMA1, LL_DMA_CHANNEL_1, LL_ADC_DMA_GetRegAddr(ADC1, LL_ADC_DMA_REG_REGULAR_DATA_MULTI), (uint32_t)&adc_dma_buffer, LL_DMA_DIRECTION_PERIPH_TO_MEMORY);
			LL_DMA_SetDataLength(  DMA1, LL_DMA_CHANNEL_1, ADC_DATA_LENGTH * 2);

			// clear all flags
			LL_DMA_ClearFlag_GI1(DMA1);

			// enable selected DMA interrupts
			LL_DMA_EnableIT_TE(DMA1, LL_DMA_CHANNEL_1);
			LL_DMA_EnableIT_HT(DMA1, LL_DMA_CHANNEL_1);
			LL_DMA_EnableIT_TC(DMA1, LL_DMA_CHANNEL_1);

			NVIC_SetPriority(DMA1_Channel1_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 3, 0));
			NVIC_EnableIRQ(  DMA1_Channel1_IRQn);
		}

		ADC_InitStruct.DataAlignment      = LL_ADC_DATA_ALIGN_RIGHT;
		ADC_InitStruct.SequencersScanMode = LL_ADC_SEQ_SCAN_DISABLE;
		LL_ADC_Init(ADC1, &ADC_InitStruct);

		ADC_CommonInitStruct.Multimode = LL_ADC_MULTI_DUAL_REG_SIM_INJ_SIM;
		LL_ADC_CommonInit(__LL_ADC_COMMON_INSTANCE(ADC1), &ADC_CommonInitStruct);

		ADC_REG_InitStruct.TriggerSource    = LL_ADC_REG_TRIG_EXT_TIM3_TRGO;
		ADC_REG_InitStruct.SequencerLength  = LL_ADC_REG_SEQ_SCAN_DISABLE;
		ADC_REG_InitStruct.SequencerDiscont = LL_ADC_REG_SEQ_DISCONT_DISABLE;
		ADC_REG_InitStruct.ContinuousMode   = LL_ADC_REG_CONV_SINGLE;
		ADC_REG_InitStruct.DMATransfer      = LL_ADC_REG_DMA_TRANSFER_UNLIMITED;
		LL_ADC_REG_Init(ADC1, &ADC_REG_InitStruct);

		// LL_ADC_SAMPLINGTIME_1CYCLES_5            1.5 + 12.5 =  14 cycles, ADC clk = 12MHz, 1.2us sample time, max  857kHz sample rate
		// LL_ADC_SAMPLINGTIME_7CYCLES_5            7.5 + 12.5 =  20 cycles, ADC clk = 12MHz, 1.7us sample time, max  600kHz sample rate
		// LL_ADC_SAMPLINGTIME_13CYCLES_5          13.5 + 12.5 =  26 cycles, ADC clk = 12MHz, 2.2us sample time, max  461kHz sample rate
		// LL_ADC_SAMPLINGTIME_28CYCLES_5          28.5 + 12.5 =  41 cycles, ADC clk = 12MHz, 3.4us sample time, max  292kHz sample rate
		// LL_ADC_SAMPLINGTIME_41CYCLES_5          41.5 + 12.5 =  54 cycles, ADC clk = 12MHz, 4.5us sample time, max  222kHz sample rate
		// LL_ADC_SAMPLINGTIME_55CYCLES_5          55.5 + 12.5 =  68 cycles, ADC clk = 12MHz, 5.7us sample time, max  176kHz sample rate
		// LL_ADC_SAMPLINGTIME_71CYCLES_5          71.5 + 12.5 =  84 cycles, ADC clk = 12MHz, 7.0us sample time, max  142kHz sample rate
		// LL_ADC_SAMPLINGTIME_239CYCLES_5        239.5 + 12.5 = 252 cycles, ADC clk = 12MHz, 21us  sample time, max 47.6kHz sample rate

		LL_ADC_REG_SetSequencerRanks( ADC1, LL_ADC_REG_RANK_1, LL_ADC_CHANNEL_0);
		LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_0,  LL_ADC_SAMPLINGTIME_71CYCLES_5);

		// *********************************
		// ADC2

		LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_ADC2);

		ADC_InitStruct.DataAlignment      = LL_ADC_DATA_ALIGN_RIGHT;
		ADC_InitStruct.SequencersScanMode = LL_ADC_SEQ_SCAN_DISABLE;
		LL_ADC_Init(ADC2, &ADC_InitStruct);

		ADC_REG_InitStruct.TriggerSource    = LL_ADC_REG_TRIG_SOFTWARE;
		ADC_REG_InitStruct.SequencerLength  = LL_ADC_REG_SEQ_SCAN_DISABLE;
		ADC_REG_InitStruct.SequencerDiscont = LL_ADC_REG_SEQ_DISCONT_DISABLE;
		ADC_REG_InitStruct.ContinuousMode   = LL_ADC_REG_CONV_CONTINUOUS;
		ADC_REG_InitStruct.DMATransfer      = LL_ADC_REG_DMA_TRANSFER_NONE;
		LL_ADC_REG_Init(ADC2, &ADC_REG_InitStruct);

		LL_ADC_REG_SetSequencerRanks( ADC2, LL_ADC_REG_RANK_1, LL_ADC_CHANNEL_1);
		LL_ADC_SetChannelSamplingTime(ADC2, LL_ADC_CHANNEL_1,  LL_ADC_SAMPLINGTIME_71CYCLES_5);

		// *********************************

		#if 1
		{	// calibrate the ADC's

			LL_ADC_Enable(ADC1);
			LL_ADC_Enable(ADC2);

			LL_mDelay(2);

			const uint32_t tick = sys_tick;
			LL_ADC_StartCalibration(ADC1);
			LL_ADC_StartCalibration(ADC2);
			while ((LL_ADC_IsCalibrationOnGoing(ADC1) || LL_ADC_IsCalibrationOnGoing(ADC2)) && (sys_tick - tick) < 100)
				__WFI();

			LL_mDelay(2);

			LL_ADC_Disable(ADC2);
			LL_ADC_Disable(ADC1);
		}
		#endif

		// *********************************

	#else
		// single ADC mode

		LL_ADC_Disable(ADC1);

		{	// setup the ADC DMA

			LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA1);

			LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_1);

			LL_DMA_SetDataTransferDirection(DMA1, LL_DMA_CHANNEL_1, LL_DMA_DIRECTION_PERIPH_TO_MEMORY);
			LL_DMA_SetChannelPriorityLevel( DMA1, LL_DMA_CHANNEL_1, LL_DMA_PRIORITY_HIGH);
			LL_DMA_SetMode(                 DMA1, LL_DMA_CHANNEL_1, LL_DMA_MODE_CIRCULAR);
			LL_DMA_SetPeriphIncMode(        DMA1, LL_DMA_CHANNEL_1, LL_DMA_PERIPH_NOINCREMENT);
			LL_DMA_SetMemoryIncMode(        DMA1, LL_DMA_CHANNEL_1, LL_DMA_MEMORY_INCREMENT);
			LL_DMA_SetPeriphSize(           DMA1, LL_DMA_CHANNEL_1, LL_DMA_PDATAALIGN_HALFWORD);
			LL_DMA_SetMemorySize(           DMA1, LL_DMA_CHANNEL_1, LL_DMA_MDATAALIGN_HALFWORD);

			LL_DMA_ConfigAddresses(DMA1, LL_DMA_CHANNEL_1, LL_ADC_DMA_GetRegAddr(ADC1, LL_ADC_DMA_REG_REGULAR_DATA), (uint32_t)&adc_dma_buffer, LL_DMA_DIRECTION_PERIPH_TO_MEMORY);
			LL_DMA_SetDataLength(  DMA1, LL_DMA_CHANNEL_1, ADC_DATA_LENGTH * 2);

			// enable selected DMA interrupts
			LL_DMA_ClearFlag_GI1(DMA1);
			LL_DMA_EnableIT_TE(DMA1, LL_DMA_CHANNEL_1);
			LL_DMA_EnableIT_HT(DMA1, LL_DMA_CHANNEL_1);
			LL_DMA_EnableIT_TC(DMA1, LL_DMA_CHANNEL_1);

			NVIC_SetPriority(DMA1_Channel1_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 3, 0));
			NVIC_EnableIRQ(  DMA1_Channel1_IRQn);
		}

		ADC_InitStruct.DataAlignment      = LL_ADC_DATA_ALIGN_RIGHT;
		ADC_InitStruct.SequencersScanMode = LL_ADC_SEQ_SCAN_ENABLE;
		LL_ADC_Init(ADC1, &ADC_InitStruct);

		ADC_CommonInitStruct.Multimode = LL_ADC_MULTI_INDEPENDENT;
		LL_ADC_CommonInit(__LL_ADC_COMMON_INSTANCE(ADC1), &ADC_CommonInitStruct);

		ADC_REG_InitStruct.TriggerSource    = LL_ADC_REG_TRIG_EXT_TIM3_TRGO;
		ADC_REG_InitStruct.SequencerLength  = LL_ADC_REG_SEQ_SCAN_ENABLE_2RANKS;
		ADC_REG_InitStruct.SequencerDiscont = LL_ADC_REG_SEQ_DISCONT_DISABLE;
		ADC_REG_InitStruct.ContinuousMode   = LL_ADC_REG_CONV_SINGLE;
		ADC_REG_InitStruct.DMATransfer      = LL_ADC_REG_DMA_TRANSFER_UNLIMITED;
		LL_ADC_REG_Init(ADC1, &ADC_REG_InitStruct);

		// LL_ADC_SAMPLINGTIME_1CYCLES_5            1.5 + 12.5 =  14 cycles, ADC clk = 12MHz, 1.2us sample time, max  857kHz sample rate
		// LL_ADC_SAMPLINGTIME_7CYCLES_5            7.5 + 12.5 =  20 cycles, ADC clk = 12MHz, 1.7us sample time, max  600kHz sample rate
		// LL_ADC_SAMPLINGTIME_13CYCLES_5          13.5 + 12.5 =  26 cycles, ADC clk = 12MHz, 2.2us sample time, max  461kHz sample rate
		// LL_ADC_SAMPLINGTIME_28CYCLES_5          28.5 + 12.5 =  41 cycles, ADC clk = 12MHz, 3.4us sample time, max  292kHz sample rate
		// LL_ADC_SAMPLINGTIME_41CYCLES_5          41.5 + 12.5 =  54 cycles, ADC clk = 12MHz, 4.5us sample time, max  222kHz sample rate
		// LL_ADC_SAMPLINGTIME_55CYCLES_5          55.5 + 12.5 =  68 cycles, ADC clk = 12MHz, 5.7us sample time, max  176kHz sample rate
		// LL_ADC_SAMPLINGTIME_71CYCLES_5          71.5 + 12.5 =  84 cycles, ADC clk = 12MHz, 7.0us sample time, max  142kHz sample rate
		// LL_ADC_SAMPLINGTIME_239CYCLES_5        239.5 + 12.5 = 252 cycles, ADC clk = 12MHz, 21us  sample time, max 47.6kHz sample rate

		LL_ADC_REG_SetSequencerRanks( ADC1, LL_ADC_REG_RANK_1, LL_ADC_CHANNEL_0);
		LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_0,  LL_ADC_SAMPLINGTIME_41CYCLES_5);

		LL_ADC_REG_SetSequencerRanks( ADC1, LL_ADC_REG_RANK_2, LL_ADC_CHANNEL_1);
		LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_1,  LL_ADC_SAMPLINGTIME_41CYCLES_5);

		#if 1
		{	// calibrate the ADC

			LL_ADC_Enable(ADC1);

			LL_mDelay(2);

			const uint32_t tick = sys_tick;
			LL_ADC_StartCalibration(ADC1);
			while (LL_ADC_IsCalibrationOnGoing(ADC1) && (sys_tick - tick) < 100)
				__WFI();

			LL_mDelay(2);

			LL_ADC_Disable(ADC1);
		}
		#endif

	#endif

	// *********************************
	// all done, start the ADC sampling

	LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_1);

	#ifdef DUAL_ADC_MODE
		LL_ADC_ClearFlag_EOS(ADC2);
	#endif
	LL_ADC_ClearFlag_EOS(ADC1);

	#ifdef DUAL_ADC_MODE
		LL_ADC_Enable(ADC2);
	#endif
	LL_ADC_Enable(ADC1);

 	#ifdef DUAL_ADC_MODE
		LL_ADC_REG_StartConversionSWStart(ADC2);
	#endif
 	LL_ADC_REG_StartConversionExtTrig(ADC1, LL_ADC_REG_TRIG_EXT_RISING);
}

// this timer clocks the ADC's and the DAC DMA
//
// so we configure the timer to run at the ADC/DAC sample rate
//
void MX_TIM3_Init(void)
{
	{	// setup the DAC DMA

		LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA1);

		LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_3);

		LL_DMA_SetDataTransferDirection(DMA1, LL_DMA_CHANNEL_3, LL_DMA_DIRECTION_MEMORY_TO_PERIPH);
		LL_DMA_SetChannelPriorityLevel( DMA1, LL_DMA_CHANNEL_3, LL_DMA_PRIORITY_VERYHIGH);
		LL_DMA_SetMode(                 DMA1, LL_DMA_CHANNEL_3, LL_DMA_MODE_CIRCULAR);
		LL_DMA_SetPeriphIncMode(        DMA1, LL_DMA_CHANNEL_3, LL_DMA_PERIPH_NOINCREMENT);
		LL_DMA_SetMemoryIncMode(        DMA1, LL_DMA_CHANNEL_3, LL_DMA_MEMORY_INCREMENT);
		LL_DMA_SetPeriphSize(           DMA1, LL_DMA_CHANNEL_3, LL_DMA_PDATAALIGN_HALFWORD);
		LL_DMA_SetMemorySize(           DMA1, LL_DMA_CHANNEL_3, LL_DMA_MDATAALIGN_HALFWORD);

		LL_DMA_ConfigAddresses(DMA1, LL_DMA_CHANNEL_3, (uint32_t)&sine_table, (uint32_t)&GPIOB->ODR, LL_DMA_DIRECTION_MEMORY_TO_PERIPH);
		LL_DMA_SetDataLength(  DMA1, LL_DMA_CHANNEL_3, ARRAY_SIZE(sine_table));
	}

	{	// setup the timer

		LL_TIM_InitTypeDef TIM_InitStruct = {0};

		LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM3);

		const uint32_t timer_rate_Hz = (ADC_DATA_LENGTH / 2) * measurement_Hz;

		TIM_InitStruct.Prescaler     = 0;
		TIM_InitStruct.CounterMode   = LL_TIM_COUNTERMODE_UP;
		TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
//		TIM_InitStruct.Autoreload    = __LL_TIM_CALC_ARR(rcc_clocks.HCLK_Frequency, TIM_InitStruct.Prescaler, timer_rate_Hz);
		TIM_InitStruct.Autoreload    = (((rcc_clocks.HCLK_Frequency / (TIM_InitStruct.Prescaler + 1)) + (timer_rate_Hz / 2)) / timer_rate_Hz) - 1;
		LL_TIM_Init(TIM3, &TIM_InitStruct);

		LL_TIM_EnableARRPreload(      TIM3);
		LL_TIM_SetClockSource(        TIM3, LL_TIM_CLOCKSOURCE_INTERNAL);
		LL_TIM_SetTriggerOutput(      TIM3, LL_TIM_TRGO_UPDATE);
		LL_TIM_DisableMasterSlaveMode(TIM3);

		// connect the timer to the DAC DMA
		LL_TIM_EnableDMAReq_UPDATE(TIM3);
	}

	// lets go already !
	LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_3);
	LL_TIM_EnableCounter(TIM3);
}

// setup the serial port UART
//
void MX_USART1_UART_Init(void)
{
	LL_USART_InitTypeDef USART_InitStruct = {0};
	LL_GPIO_InitTypeDef  GPIO_InitStruct  = {0};

	LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_USART1);
	LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOA);

	GPIO_InitStruct.Pin        = UART1_TXD_Pin;
	GPIO_InitStruct.Mode       = LL_GPIO_MODE_ALTERNATE;
	GPIO_InitStruct.Speed      = LL_GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	LL_GPIO_Init(UART1_TXD_GPIO_Port, &GPIO_InitStruct);

	GPIO_InitStruct.Pin        = UART1_RXD_Pin;
	GPIO_InitStruct.Mode       = LL_GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull       = LL_GPIO_PULL_UP;
	LL_GPIO_Init(UART1_RXD_GPIO_Port, &GPIO_InitStruct);

	{	// TX DMA

		LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA1);

		LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_4);

		LL_DMA_SetDataTransferDirection(DMA1, LL_DMA_CHANNEL_4, LL_DMA_DIRECTION_MEMORY_TO_PERIPH);
		LL_DMA_SetChannelPriorityLevel( DMA1, LL_DMA_CHANNEL_4, LL_DMA_PRIORITY_LOW);
		LL_DMA_SetMode(                 DMA1, LL_DMA_CHANNEL_4, LL_DMA_MODE_NORMAL);
		LL_DMA_SetPeriphIncMode(        DMA1, LL_DMA_CHANNEL_4, LL_DMA_PERIPH_NOINCREMENT);
		LL_DMA_SetMemoryIncMode(        DMA1, LL_DMA_CHANNEL_4, LL_DMA_MEMORY_INCREMENT);
		LL_DMA_SetPeriphSize(           DMA1, LL_DMA_CHANNEL_4, LL_DMA_PDATAALIGN_BYTE);
		LL_DMA_SetMemorySize(           DMA1, LL_DMA_CHANNEL_4, LL_DMA_MDATAALIGN_BYTE);

		// enable selected interrupts
		LL_DMA_ClearFlag_GI4(DMA1);
		LL_DMA_EnableIT_TE(DMA1, LL_DMA_CHANNEL_4);
		LL_DMA_EnableIT_TC(DMA1, LL_DMA_CHANNEL_4);

		NVIC_SetPriority(DMA1_Channel4_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 1, 0));
		NVIC_EnableIRQ(DMA1_Channel4_IRQn);
	}

	USART_InitStruct.BaudRate            = settings.baudrate;
	USART_InitStruct.DataWidth           = LL_USART_DATAWIDTH_8B;
	USART_InitStruct.StopBits            = LL_USART_STOPBITS_1;
	USART_InitStruct.Parity              = LL_USART_PARITY_NONE;
	USART_InitStruct.TransferDirection   = LL_USART_DIRECTION_TX_RX;
	USART_InitStruct.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
	USART_InitStruct.OverSampling        = LL_USART_OVERSAMPLING_16;
	LL_USART_Init(USART1, &USART_InitStruct);

	LL_USART_ConfigAsyncMode(USART1);

	// connect the UART to the DMA
	LL_USART_EnableDMAReq_TX(USART1);
	//LL_USART_EnableDMAReq_RX(USART1);

	// clear RX error flags .. clearing any one of these also clears all the others
	LL_USART_ClearFlag_ORE(USART1);    // OverRun Error Flag
	//LL_USART_ClearFlag_PE(USART1);   // Parity Error Flag
	//LL_USART_ClearFlag_NE(USART1);   // Noise detected Flag
	//LL_USART_ClearFlag_FE(USART1);   // Framing Error Flag
	//LL_USART_ClearFlag_IDLE(USART1); // IDLE line detected Flag

	// RX isn't done by DMA
	LL_USART_ClearFlag_RXNE(USART1);
	LL_USART_EnableIT_RXNE(USART1);

	NVIC_SetPriority(USART1_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 1, 0));
	NVIC_EnableIRQ(USART1_IRQn);

	LL_USART_Enable(USART1);
}

// setup various GPIO pins
//
void MX_GPIO_Init(void)
{
	LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

	LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOA);
	LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOB);
	LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOC);
	LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOD);

	// *****************
	// output pins

	LL_GPIO_ResetOutputPin(TP21_GPIO_Port, TP21_Pin);
	LL_GPIO_ResetOutputPin(TP22_GPIO_Port, TP22_Pin);
	LL_GPIO_ResetOutputPin(LED_GPIO_Port,  LED_Pin);
	LL_GPIO_ResetOutputPin(GS_GPIO_Port,   GS_Pin);
	LL_GPIO_ResetOutputPin(VI_GPIO_Port,   VI_Pin);

	LL_GPIO_ResetOutputPin(GPIOB, DA0_Pin | DA1_Pin | DA2_Pin | DA3_Pin | DA4_Pin | DA5_Pin | DA6_Pin | DA7_Pin);

	GPIO_InitStruct.Mode       = LL_GPIO_MODE_OUTPUT;
	GPIO_InitStruct.Speed      = LL_GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	GPIO_InitStruct.Pull       = LL_GPIO_PULL_UP;
	GPIO_InitStruct.Pin        = TP21_Pin;
	LL_GPIO_Init(TP21_GPIO_Port, &GPIO_InitStruct);
	GPIO_InitStruct.Pin        = TP22_Pin;
	LL_GPIO_Init(TP22_GPIO_Port, &GPIO_InitStruct);
	GPIO_InitStruct.Pin        = LED_Pin;
	LL_GPIO_Init(LED_GPIO_Port,  &GPIO_InitStruct);
	GPIO_InitStruct.Pin        = GS_Pin;
	LL_GPIO_Init(GS_GPIO_Port,   &GPIO_InitStruct);
	GPIO_InitStruct.Pin        = VI_Pin;
	LL_GPIO_Init(VI_GPIO_Port,   &GPIO_InitStruct);
	GPIO_InitStruct.Speed      = LL_GPIO_SPEED_FREQ_HIGH;
	GPIO_InitStruct.Pin        = DA0_Pin | DA1_Pin | DA2_Pin | DA3_Pin | DA4_Pin | DA5_Pin | DA6_Pin | DA7_Pin;
	LL_GPIO_Init(GPIOB,          &GPIO_InitStruct);
	GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	GPIO_InitStruct.Pin        = SW_I2C_SCL_Pin;
	LL_GPIO_Init(SW_I2C_SCL_GPIO_Port, &GPIO_InitStruct);
	GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_OPENDRAIN;
	GPIO_InitStruct.Pin        = SW_I2C_SDA_Pin;
	LL_GPIO_Init(SW_I2C_SDA_GPIO_Port, &GPIO_InitStruct);

	// *****************
	// input pins

	GPIO_InitStruct.Mode       = LL_GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull       = LL_GPIO_PULL_UP;
	GPIO_InitStruct.Pin        = BUTT_HOLD_Pin;
	LL_GPIO_Init(BUTT_HOLD_GPIO_Port, &GPIO_InitStruct);
	GPIO_InitStruct.Pin        = BUTT_SP_Pin;
	LL_GPIO_Init(BUTT_SP_GPIO_Port,   &GPIO_InitStruct);
	GPIO_InitStruct.Pin        = BUTT_RCL_Pin;
	LL_GPIO_Init(BUTT_RCL_GPIO_Port,  &GPIO_InitStruct);

	// protect the button input pins from the DAC DMA, it corrupts the upper 8-bits of port-B :(
	// note, this does not work, it's broken on this cpu :(
	LL_GPIO_LockPin(BUTT_HOLD_GPIO_Port, BUTT_HOLD_Pin);
	LL_GPIO_LockPin(BUTT_SP_GPIO_Port,   BUTT_SP_Pin);
	LL_GPIO_LockPin(BUTT_RCL_GPIO_Port,  BUTT_RCL_Pin);

	// *****************
}

#ifdef USE_IWDG

	// feed the doggy
	//
	void service_IWDG(const bool force_update)
	{
		const uint32_t reload_sec = iwdg_timeout_sec / 2;

		if (iwdg_tick >= (1000 * reload_sec) || force_update)
		{
			iwdg_tick = 0;
			LL_IWDG_ReloadCounter(IWDG);
		}
	}

	// initialise the internal watchdog
	//
	void MX_IWDG_Init(void)
	{
		const uint32_t prescaler_div = LL_IWDG_PRESCALER_256;     // 4, 8, 16, 32, 64, 128 or 256

		uint32_t clk_Hz = LSI_VALUE;
		switch (prescaler_div)
		{
			default:
			case LL_IWDG_PRESCALER_4:
				clk_Hz >>= 2;
				break;
			case LL_IWDG_PRESCALER_8:
				clk_Hz >>= 3;
				break;
			case LL_IWDG_PRESCALER_16:
				clk_Hz >>= 4;
				break;
			case LL_IWDG_PRESCALER_32:
				clk_Hz >>= 5;
				break;
			case LL_IWDG_PRESCALER_64:
				clk_Hz >>= 6;
				break;
			case LL_IWDG_PRESCALER_128:
				clk_Hz >>= 7;
				break;
			case LL_IWDG_PRESCALER_256:
				clk_Hz >>= 8;
				break;
		}

		LL_IWDG_Enable(IWDG);

		LL_IWDG_EnableWriteAccess(IWDG);
		{
			const uint32_t reload_tick = IWDG_RLR_RL_Msk;

			iwdg_timeout_sec = reload_tick / clk_Hz;

			LL_IWDG_SetPrescaler(    IWDG, prescaler_div);
			LL_IWDG_SetReloadCounter(IWDG, reload_tick);

			while (!LL_IWDG_IsReady(IWDG)) {}
		}
		LL_IWDG_DisableWriteAccess(IWDG);

		service_IWDG(true);
	}

#endif

// *******************************************

void Error_Handler(void)
{
	stop(400);
}

void NMI_Handler(void)
{
	stop(300);
}

void HardFault_Handler(void)
{
	stop(100);
}

void MemManage_Handler(void)
{
	stop(100);
}

void BusFault_Handler(void)
{
	stop(100);
}

void UsageFault_Handler(void)
{
	stop(100);
}

void SVC_Handler(void)
{
}

void DebugMon_Handler(void)
{
}

void PendSV_Handler(void)
{
}

// 1ms system interrupt
//
void SysTick_Handler(void)
{
	HAL_IncTick();

	sys_tick++;

	#if 1
	{	// debounce the push buttons

		const uint32_t tick = sys_tick;

		const int16_t debounce_ms = 30;

		for (unsigned int i = 0; i < BUTTON_NUM; i++)
		{
			t_button *butt = &button[i];
			if (butt->gpio_port == NULL)
				continue;

			// update debounce counter
			int16_t debounce = butt->debounce;
			debounce = (LL_GPIO_IsInputPinSet(butt->gpio_port, butt->gpio_pin) == 0) ? debounce + 1 : debounce - 1;
			debounce = (debounce < 0) ? 0 : (debounce > debounce_ms) ? debounce_ms : debounce;
			butt->debounce = debounce;

			const uint32_t pressed_ms = butt->pressed_ms;

			if (pressed_ms == 0 && debounce >= debounce_ms)
			{	// just pressed
				butt->released   = 0;                             // clear released flag
				butt->held_ms    = 0;                             // reset held down time
				butt->processed  = 0;                             // clear flag
				butt->pressed_ms = tick;                          // remember the tick the button was pressed
			}
			else
			if (pressed_ms > 0 && debounce <= 0)
			{	// just released
				if (butt->processed)
				{
					butt->held_ms    = 0;
					butt->released   = 0;
					butt->pressed_ms = 0;
					butt->processed  = 0;
				}
				else
				{
					butt->held_ms    = tick - pressed_ms;         // save the time the button was held down for
					butt->released   = 1;                         // set released flag
					butt->pressed_ms = 0;                         // reset pressed tick
				}
			}
			else
			if (pressed_ms > 0) // && butt->processed == 0)
			{
				butt->held_ms = tick - pressed_ms;                // time the button has been held down for
			}
			else
			if (butt->processed)
			{
				butt->held_ms    = 0;
				butt->released   = 0;
				butt->pressed_ms = 0;
				butt->processed  = 0;
			}
		}
	}
	#endif

	if (save_settings_timer > 0)
		save_settings_timer--;

	if (serial.rx.timer < (serial.rx.timer + 1))         // prevent roll-over
		serial.rx.timer++;

	#ifdef USE_IWDG
		iwdg_tick++;
	#endif
}

// ADC DMA
// pass the ADC sample blocks over to the routine that does something with them
//
void DMA1_Channel1_IRQHandler(void)
{
	if (LL_DMA_IsActiveFlag_TE1(DMA1))
	{

		LL_DMA_ClearFlag_TE1(DMA1);
	}

	if (LL_DMA_IsActiveFlag_HT1(DMA1))
	{
		process_ADC_DMA(&adc_dma_buffer[0]);  // lower half of ADC buffer
		LL_DMA_ClearFlag_HT1(DMA1);
	}

	if (LL_DMA_IsActiveFlag_TC1(DMA1))
	{
		process_ADC_DMA(&adc_dma_buffer[1]);  // upper half of ADC buffer
		LL_DMA_ClearFlag_TC1(DMA1);
	}
}

/*
// DAC DMA
void DMA1_Channel3_IRQHandler(void)
{
	if (LL_DMA_IsActiveFlag_TC3(DMA1))
	{

		LL_DMA_ClearFlag_TC3(DMA1);
	}
}
*/

// UART TX DMA
// the serial TX DMA has completed, disable it ready for the next serial send
//
void DMA1_Channel4_IRQHandler(void)
{
	// disable the TX DMA if the send has completed
	if (LL_DMA_IsActiveFlag_TE4(DMA1) || LL_DMA_IsActiveFlag_TC4(DMA1))
	{
		LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_4);
		LL_DMA_ClearFlag_TE4(DMA1);
		LL_DMA_ClearFlag_TC4(DMA1);
	}
}

// serial RX
//
void USART1_IRQHandler(void)
{
	{	// RX

		//if (LL_USART_IsActiveFlag_IDLE(USART1))
		//	LL_USART_ClearFlag_IDLE(USART1);

		register uint32_t wr = serial.rx.buffer_wr;

		while (LL_USART_IsActiveFlag_RXNE(USART1))
		{
			serial.rx.buffer[wr] = LL_USART_ReceiveData8(USART1);      // save new received byte
			wr = (++wr >= sizeof(serial.rx.buffer)) ? 0 : wr;          // update write index
			serial.rx.buffer_wr = wr;                                  // save it

			serial.rx.timer     = 0;                                   // reset RX time-out timer

			if (wr == serial.rx.buffer_rd)
			{	// RX buffer overrun
			}
		}

		// clear RX error flags .. clearing any one of these also clears all the others
		LL_USART_ClearFlag_ORE(USART1);    // OverRun Error Flag
		//LL_USART_ClearFlag_PE(USART1);   // Parity Error Flag
		//LL_USART_ClearFlag_NE(USART1);   // Noise detected Flag
		//LL_USART_ClearFlag_FE(USART1);   // Framing Error Flag
		//LL_USART_ClearFlag_IDLE(USART1); // IDLE line detected Flag
	}

	#if 0
	{	// TX

		const uint8_t enabled = LL_USART_IsEnabledIT_TXE(USART1);
		//if (enabled)
		{
			const uint32_t wr = serial.tx.buffer_wr;
			      uint32_t rd = serial.tx.buffer_rd;

			while (rd != wr && LL_USART_IsActiveFlag_TXE(USART1))
			{	// send the next byte
				LL_USART_TransmitData8(USART1, serial.tx.buffer[rd]);
				rd = (++rd >= sizeof(serial.tx.buffer)) ? 0 : rd;
				serial.tx.buffer_rd = rd;

				serial.tx.timer     = 0;
			}

			if (rd == wr)
			{
				if (enabled)
					LL_USART_DisableIT_TXE(USART1);        // all done
			}
			else
			{
				if (!enabled)
					LL_USART_EnableIT_TXE(USART1);         // enable interrupt to continue sending
			}
		}
	}
	#endif
}

#ifdef  USE_FULL_ASSERT
	void assert_failed(uint8_t *file, uint32_t line)
	{
	}
#endif

// ***********************************************************

// process any button presses
//
void process_buttons(void)
{
	// both HOLD and S/P buttons held down
	if (button[BUTTON_HOLD].pressed_ms > 0 && button[BUTTON_SP].pressed_ms > 0 && button[BUTTON_RCL].pressed_ms == 0)
	{
		if (button[BUTTON_HOLD].held_ms >= 800 && button[BUTTON_SP].held_ms >= 800)
		{	// clear all saved settings (inc open/short calibrations), then reboot
			clear_settings();
			reboot();
		}
		return;
	}

	if (op_mode != OP_MODE_MEASURING)
	{	// busy calibrating

		// ignore any button presses during calibration
		for (unsigned int i = 0; i < BUTTON_NUM; i++)
			button[i].processed = 1;

		return;
	}

	// *************
	// HOLD button

	if (button[BUTTON_HOLD].pressed_ms > 0 && button[BUTTON_SP].pressed_ms == 0 && button[BUTTON_RCL].pressed_ms == 0)
	{
		if (button[BUTTON_HOLD].held_ms >= 500 && button[BUTTON_HOLD].processed == 0)
		{	// HOLD held down
			button[BUTTON_HOLD].processed = 1;

			display_hold = 0;
			memset((void *)&calibrate, 0, sizeof(calibrate));
			op_mode = OP_MODE_OPEN_PROBE_CALIBRATION;
			set_measurement_frequency(100);
			draw_screen();
		}
		return;
	}

	if (button[BUTTON_HOLD].released)
	{
		if (button[BUTTON_HOLD].processed == 0)
		{
			button[BUTTON_HOLD].processed = 1;

			display_hold ^= 1u;                         // toggle HOLD flag
			draw_screen();
		}
		return;
	}

	// *************
	// S/P button

	if (button[BUTTON_HOLD].pressed_ms == 0 && button[BUTTON_SP].pressed_ms > 0 && button[BUTTON_RCL].pressed_ms == 0)
	{
		if (button[BUTTON_SP].held_ms >= 500 && button[BUTTON_SP].processed == 0)
		{	// S/P held down
			button[BUTTON_SP].processed = 1;

			display_hold = 0;
			memset((void *)&calibrate, 0, sizeof(calibrate));
			op_mode = OP_MODE_SHORTED_PROBE_CALIBRATION;
			set_measurement_frequency(100);
			draw_screen();
		}
		return;
	}

	if (button[BUTTON_SP].released)
	{
		if (button[BUTTON_SP].processed == 0)
		{
			button[BUTTON_SP].processed = 1;

			display_hold = 0;
			settings.flags ^= SETTING_FLAG_PARALLEL;    // toggle Serial/Parallel display
			save_settings_timer = SAVE_SETTINGS_MS;     // save settings

			draw_screen();
		}
		return;
	}

	// *************
	// RCL button

	if (button[BUTTON_HOLD].pressed_ms == 0 && button[BUTTON_SP].pressed_ms == 0 && button[BUTTON_RCL].pressed_ms > 0)
	{
		if (button[BUTTON_RCL].held_ms >= 500 && button[BUTTON_RCL].processed == 0)
		{	// RCL held down
			button[BUTTON_RCL].processed = 1;

			display_hold = 0;

			// cycle the frequency
			switch (settings.measurement_Hz)
			{
				default:
				case 100:
					settings.measurement_Hz = 1000;
					break;
				case 1000:
					settings.measurement_Hz = 100;
					break;
			}

			set_measurement_frequency(settings.measurement_Hz);

			// save settings
			save_settings_timer = SAVE_SETTINGS_MS;

			draw_screen();
		}
		return;
	}

	if (button[BUTTON_RCL].released)
	{
		if (button[BUTTON_RCL].processed == 0)
		{
			button[BUTTON_RCL].processed = 1;

			display_hold = 0;

			// cycle through the LCR modes (inc SLOW/FAST mode)
			settings.flags ^= SETTING_FLAG_FAST_UPDATES;
			if (!(settings.flags & SETTING_FLAG_FAST_UPDATES))
			{
				unsigned int mode = settings.lcr_mode;
				//if (++mode > LCR_MODE_AUTO)               // TODO:
				if (++mode > LCR_MODE_RESISTANCE)
					mode = LCR_MODE_INDUCTANCE;
				settings.lcr_mode = mode;
			}

			// save settings
			save_settings_timer = SAVE_SETTINGS_MS;

			draw_screen();
		}
		return;
	}

	// *************
}

// send data out via the serial port
//
// we pass the data over to the serial TX DMA for it to send out
//
void send_data(void)
{
	if ((settings.flags & SETTING_FLAG_UART_DSO) == 0)
		return;

	if (LL_DMA_IsEnabledChannel(DMA1, LL_DMA_CHANNEL_4))
		return;     // uart DMA is still busy sending

	// send the sampled data down the serial link

	if (settings.flags & SETTING_FLAG_SEND_BINARY)
	{	// send as binary packet

		// create TX packet
		tx_packet.marker = PACKET_MARKER;                                         // packet start marker
		memcpy(tx_packet.data, &adc_data, sizeof(adc_data));                      // packet data
		tx_packet.crc = CRC16_block(0, tx_packet.data, sizeof(tx_packet.data));   // packet CRC - compute the CRC of the data

		// sen dit
		start_tx_dma(&tx_packet, sizeof(tx_packet));
	}
	else
	{	// send as ASCII
		const unsigned int cols = 8;
		for (unsigned int i = 0; i < ADC_DATA_LENGTH; i++)
		{
			printf("%3u,", 1 + i);
			for (unsigned int col = 0; col < (cols - 1); col++)
				printf("%0.1f,", adc_data[col][i]);
			printf("%0.1f" NEWLINE, adc_data[cols - 1][i]);
		}
	}
}

enum t_cmd_id : uint8_t {
	CMD_NONE_ID = 0,
	CMD_HELP_ID1,
	CMD_HELP_ID2,
	CMD_BAUDRATE_ID,
	CMD_DEFAULTS_ID,
	CMD_OPEN_CAL_ID,
	CMD_SHORT_CAL_ID,
	CMD_DATA_ID,
	CMD_HOLD_ID,
	CMD_FREQUENCY_ID,
	CMD_LCR_MODE_ID,
	CMD_SP_MODE_ID,
	CMD_SERIES_ID,
	CMD_PARALLEL_ID,
	CMD_REBOOT_ID,
	CMD_VERSION_ID
};

typedef struct {
	const char    *token;
	const char    *description;
	const t_cmd_id id;
} t_cmd;

// serial command table
const t_cmd cmds[] = {
	{"?",         "              .. this help",                       CMD_HELP_ID1    },
	{"help",      "              .. this help",                       CMD_HELP_ID2    },
	{"baudrate",  "[baudrate]    .. read/set serial baudrate",        CMD_BAUDRATE_ID },
	{"data",      "[off/asc/bin] .. read/set sending real-time data", CMD_DATA_ID     },
	{"frequency", "[Hz]          .. read/set measurement frequency",  CMD_FREQUENCY_ID},
	{"hold",      "              .. toggle display hold on/off",      CMD_HOLD_ID     },
	{"lcrmode",   "[r/l/i/c]     .. read/set LCR mode",               CMD_LCR_MODE_ID },
	{"spmode",    "[s/p]         .. read/set Series/Parallel mode",   CMD_SP_MODE_ID  },
	{"opencal",   "              .. run open probe calibration",      CMD_OPEN_CAL_ID },
	{"shortcal",  "              .. run shorted probe calibration",   CMD_SHORT_CAL_ID},
	{"reboot",    "              .. reboot this unit",                CMD_REBOOT_ID   },
	{"defaults",  "              .. restore defaults",                CMD_DEFAULTS_ID },
	{"version",   "              .. this units version",              CMD_VERSION_ID  },
	{NULL,        "",                                                 CMD_NONE_ID     }    // last one, DO NOT delete this
};

// process any received serial commands
//
void process_serial_command(char cmd[], unsigned int len)
{
	if (cmd == NULL || len == 0)
		return;

	// ****************
	// clean up the text

	// replace any tabs with spaces
	// and lower case all
	for (unsigned int i = 0; i < len; i++)
		cmd[i] = (cmd[i] == '\t') ? ' ' : tolower(cmd[i]);

	// trim leading
	while (len > 0 && cmd[0] <= ' ')
		memmove(cmd, cmd + 1, --len);

	// trim trailing
	while (len > 0 && cmd[len - 1] <= ' ')
		cmd[--len] = '\0';

	if (len == 0)
		return;

	// ensure it's null terminated
	cmd[len] = '\0';

	// ****************

	// determine the length of the command
	// also lowercase the command
	unsigned int param_pos = 0;
	while (param_pos < len && cmd[param_pos] > ' ')
	{
//		cmd[param_pos] = tolower(cmd[param_pos]);
		param_pos++;
	}

	// null term the command
	cmd[param_pos] = '\0';

	// determine the length of any present param
	const unsigned int param_len = (len >= (param_pos + 1)) ? len - (param_pos + 1) : 0;

	// **********

	// determine what the command is
	t_cmd_id cmd_id = CMD_NONE_ID;
	for (unsigned int i = 0; cmds[i].token != NULL && cmds[i].id != CMD_NONE_ID; i++)
	{
		if (strncmp(cmd, cmds[i].token, param_pos) != 0)
			continue;

		if (cmd_id != CMD_NONE_ID)
		{	// matches more than 1 command, treat it as an unknown command
			cmd_id = CMD_NONE_ID;
			break;
		}

		cmd_id = cmds[i].id;
	}

	// point to start of param
	param_pos = (param_len > 0) ? param_pos + 1 : param_pos;
	const char *param = &cmd[param_pos];

	// **********
	// process the command

	switch (cmd_id)
	{
		case CMD_HELP_ID1:
		case CMD_HELP_ID2:
		{
			unsigned int cmd_max_len = 0;
			for (unsigned int i = 0; cmds[i].token != NULL && cmds[i].id != CMD_NONE_ID; i++)
			{
				const unsigned int cmd_len = strlen(cmds[i].token);
				cmd_max_len = (cmd_max_len < cmd_len) ? cmd_len : cmd_max_len;
			}

			printf(NEWLINE "Commands [params] (case insensitive, can be shortened) .." NEWLINE);
			for (unsigned int i = 0; cmds[i].token != NULL; i++)
				printf("  %-*s %s" NEWLINE, cmd_max_len, cmds[i].token, cmds[i].description);

			return;
		}

		case CMD_DEFAULTS_ID:
			printf(NEWLINE "restoring defaults .." NEWLINE);
			clear_settings();
			reboot();
			break;

		case CMD_BAUDRATE_ID:
			if (param_len > 0)
			{
				char     *endptr = NULL;
				const int val    = strtol(param, &endptr, 10);
				if (errno > 0 || param == endptr || val < UART_BAUDRATE_MIN || val > UART_BAUDRATE_MAX)
				{
					printf(NEWLINE "error: baudrate param '%s'" NEWLINE, param);
					return;
				}

				const uint32_t baudrate = (val < UART_BAUDRATE_MIN) ? UART_BAUDRATE_MIN : (val > UART_BAUDRATE_MAX) ? UART_BAUDRATE_MAX : val;
				if (settings.baudrate != baudrate)
				{
					settings.baudrate = baudrate;
					LL_USART_SetBaudRate(USART1, rcc_clocks.PCLK2_Frequency, settings.baudrate);

					save_settings_timer = SAVE_SETTINGS_MS;
					draw_screen();
				}
			}

			printf(NEWLINE "baudrate %lu" NEWLINE, settings.baudrate);
			return;

		case CMD_OPEN_CAL_ID:
			if (op_mode != OP_MODE_MEASURING)
			{
				printf(NEWLINE "busy" NEWLINE);
				return;
			}

			printf(NEWLINE "open probe calibration .." NEWLINE);

			display_hold = 0;
			memset((void *)&calibrate, 0, sizeof(calibrate));
			op_mode = OP_MODE_OPEN_PROBE_CALIBRATION;
			set_measurement_frequency(100);
			draw_screen();

			return;

		case CMD_SHORT_CAL_ID:
			if (op_mode != OP_MODE_MEASURING)
			{
				printf(NEWLINE "busy" NEWLINE);
				return;
			}

			printf(NEWLINE "shorted probe calibration .." NEWLINE);

			display_hold = 0;
			memset((void *)&calibrate, 0, sizeof(calibrate));
			op_mode = OP_MODE_SHORTED_PROBE_CALIBRATION;
			set_measurement_frequency(100);
			draw_screen();

			return;

		case CMD_FREQUENCY_ID:
			if (param_len > 0)
			{
				char     *endptr = NULL;
				const int val    = strtol(param, &endptr, 10);
				if (errno > 0 || param == endptr || val < MEASURE_HZ_MIN || val > MEASURE_HZ_MAX)
				{
					printf(NEWLINE "error: frequency param '%s'" NEWLINE, param);
					return;
				}

				const uint16_t Hz = (val < MEASURE_HZ_MIN) ? MEASURE_HZ_MIN : (val > MEASURE_HZ_MAX) ? MEASURE_HZ_MAX : val;
				if (settings.measurement_Hz != Hz)
				{
					settings.measurement_Hz = Hz;
					if (op_mode == OP_MODE_MEASURING)
						set_measurement_frequency(Hz);

					save_settings_timer = SAVE_SETTINGS_MS;
					draw_screen();
				}
			}

			if (settings.measurement_Hz < 1000)
				snprintf(str_buf, sizeof(str_buf), " %u Hz" NEWLINE, settings.measurement_Hz);
			else
				snprintf(str_buf, sizeof(str_buf), " %0.3f kHz", settings.measurement_Hz * 1e-3f);
			trim_trailing_zeros(str_buf);
			printf(NEWLINE "measurement frequency %s" NEWLINE, str_buf);

			return;

		case CMD_LCR_MODE_ID:
			if (param_len > 0)
			{
				if (strncmp(param, "resistance", param_len) == 0)
					settings.lcr_mode = LCR_MODE_RESISTANCE;
				else
				if (strncmp(param, "inductance", param_len) == 0 || strncmp(param, "lenz", param_len) == 0)
					settings.lcr_mode = LCR_MODE_INDUCTANCE;
				else
				if (strncmp(param, "capacitance", param_len) == 0)
					settings.lcr_mode = LCR_MODE_CAPACITANCE;
				else
//				if (strncmp(param, "auto", param_len) == 0)
//					settings.lcr_mode = LCR_MODE_AUTO;               // todo:
//				else
				{
					printf(NEWLINE "error: mode param '%s'" NEWLINE, param);
					return;
				}

				save_settings_timer = SAVE_SETTINGS_MS;
				draw_screen();
			}

			printf(NEWLINE "lcr mode");
			fflush(NULL);
			switch (settings.lcr_mode)
			{
				case LCR_MODE_INDUCTANCE:
					printf(" inductance" NEWLINE);
					break;
				case LCR_MODE_CAPACITANCE:
					printf(" capacitance" NEWLINE);
					break;
				case LCR_MODE_RESISTANCE:
					printf(" resistance" NEWLINE);
					break;
				case LCR_MODE_AUTO:
					printf(" auto" NEWLINE);
					break;
				default:
					printf(" ERROR" NEWLINE);
					break;
			}

			return;

		case CMD_SP_MODE_ID:
			if (param_len > 0)
			{
				if (strncmp(param, "series", param_len) == 0)
					settings.flags &= ~SETTING_FLAG_PARALLEL;
				else
				if (strncmp(param, "parallel", param_len) == 0)
					settings.flags |= SETTING_FLAG_PARALLEL;
				else
				{
					printf(NEWLINE "error: spmode param '%s'" NEWLINE, param);
					return;
				}

				display_hold = 0;
				save_settings_timer = SAVE_SETTINGS_MS;
				draw_screen();
			}

			printf(NEWLINE "sp mode");
			fflush(NULL);
			if (settings.flags & SETTING_FLAG_PARALLEL)
				printf(" parallel" NEWLINE);
			else
				printf(" series" NEWLINE);

			return;

		case CMD_DATA_ID:
			if (param_len > 0)
			{
				if (strncmp(param, "off", param_len) == 0)
					settings.flags &= ~SETTING_FLAG_UART_DSO;
				else
				if (strncmp(param, "bin", param_len) == 0)
					settings.flags |= SETTING_FLAG_SEND_BINARY | SETTING_FLAG_UART_DSO;
				else
				if (strncmp(param, "asc", param_len) == 0)
					settings.flags = (settings.flags & ~SETTING_FLAG_SEND_BINARY) | SETTING_FLAG_UART_DSO;
				else
				{
					printf(NEWLINE "error: data param '%s'" NEWLINE, param);
					return;
				}

				save_settings_timer = SAVE_SETTINGS_MS;
				draw_screen();
			}

			printf(NEWLINE "live data");
			fflush(NULL);
			if (settings.flags & SETTING_FLAG_UART_DSO)
				printf(" off" NEWLINE);
			else
			if (settings.flags & SETTING_FLAG_SEND_BINARY)
				printf(" binary" NEWLINE);
			else
				printf(" ascii" NEWLINE);

			return;

		case CMD_HOLD_ID:
			if (op_mode != OP_MODE_MEASURING)
			{
				printf(NEWLINE "busy" NEWLINE);
				return;
			}

			display_hold ^= 1u;        // toggle hold flag
			draw_screen();

			printf(NEWLINE "display");
			fflush(NULL);
			if (display_hold)
				printf(" hold" NEWLINE);
			else
				printf(" run" NEWLINE);

			return;

		case CMD_REBOOT_ID:
			printf(NEWLINE "rebooting .." NEWLINE);
			reboot();
			return;

		case CMD_VERSION_ID:
			printf(NEWLINE "M181 LCR Meter v%0.2f %s %s %s %s %s %s %s" NEWLINE,
				FW_VERSION,
				reset_cause.por    ? "POR"    : "por",
				reset_cause.pin    ? "PIN"    : "pin",
				reset_cause.sft    ? "SFT"    : "sft",
				reset_cause.iwdg   ? "IWDG"   : "iwdg",
				reset_cause.wwdg   ? "WWDG"   : "wwdg",
				reset_cause.lpwr   ? "LPWR"   : "lpwr",
				reset_cause.lsirdy ? "LSIRDY" : "lsirdy");
			return;

		default:
		case CMD_NONE_ID:
			printf(NEWLINE "error: unknown command '%s'" NEWLINE, cmd);
			return;
	}
}

// extract text lines from the received serial data
//
void process_uart_receive(void)
{
	const uint32_t buf_wr = serial.rx.buffer_wr;
	uint32_t       buf_rd = serial.rx.buffer_rd;

	if (serial.rx.timer >= 3000 && (buf_rd != buf_wr || serial.rx.line.buffer_wr > 0))
	{	// serial RX time-out
		serial.rx.buffer_rd = buf_rd = buf_wr;   // drop any received data in the buffer
		serial.rx.line.buffer_wr = 0;            //   "        "
		return;
	}

	if (buf_rd == buf_wr)
		return;                          // no new RX data to process

	// process the new RX'ed data

	const uint32_t rx_buf_size   = ARRAY_SIZE(serial.rx.buffer);
	const uint32_t line_buf_size = ARRAY_SIZE(serial.rx.line.buffer);
	uint32_t       line_wr       = serial.rx.line.buffer_wr;

	while (buf_rd != buf_wr)
	{
		// number of bytes in our RX buffer waiting to be processed
		uint32_t num = (buf_wr >= buf_rd) ? buf_wr - buf_rd : rx_buf_size - buf_rd;
		if (num == 0)
			break;                       // hmm, no new data ??

		// limit to fit into our RX text line buffer
		num = (num > (line_buf_size - line_wr)) ? line_buf_size - line_wr : num;
		if (num == 0)
		{	// no room left, somethings not right, clear everything
			serial.rx.buffer_rd = buf_rd = buf_wr;
			serial.rx.line.buffer_wr = line_wr = 0;
			return;
		}

		// copy the RX'ed data into our RX text line buffer
		memcpy((void *)&serial.rx.line.buffer[line_wr], (void *)&serial.rx.buffer[buf_rd], num);
		serial.rx.line.buffer_wr = line_wr = line_wr + num;

		// update the read index
		buf_rd += num;
		serial.rx.buffer_rd = buf_rd = (buf_rd >= rx_buf_size) ? 0 : buf_rd;

		// ********************
		// process the rx'ed text line

		// find the 1st LF or CR (end of text line)
		char *p = (char *)serial.rx.line.buffer;
		while (p < ((char *)serial.rx.line.buffer + line_wr) && *p != '\r' && *p != '\n' && *p != '\0')
			p++;
		if (p >= ((char *)serial.rx.line.buffer + line_wr))
		{	// no LF or CR found
			if (line_wr >= (line_buf_size - 1))           // buffer full ?
				serial.rx.line.buffer_wr = line_wr = 0;   // yes, discard the entire text line buffer
			continue;
		}

		// found a LF and/or CR in the text line buffer - process the new text line

		// reset RX time-out timer
		serial.rx.timer = 0;

		const unsigned int pos = (unsigned int)(p - (char *)serial.rx.line.buffer);  // position of 1st CR or LF found
		if (pos > 0)
		{
			// null terminate the text line
			serial.rx.line.buffer[pos] = '\0';

			// process the new text line
			process_serial_command((char *)serial.rx.line.buffer, pos);
		}

		// reset the text line buffer
		serial.rx.line.buffer_wr = line_wr = 0;
	}
}

// do stuff
//
void process_op_mode(void)
{
	switch (op_mode)
	{
		default:
		case OP_MODE_MEASURING:                // normal measurement mode
			break;

		case OP_MODE_OPEN_PROBE_CALIBRATION:   // doing an OPEN probe calibration
		{
			// sum a number of sample block magnitudes
			for (unsigned int i = 0; i < ARRAY_SIZE(calibrate.mag_sum); i++)
				calibrate.mag_sum[i] += mag_rms[i];

			// sum a number of sample block phases
			for (unsigned int i = 0; i < ARRAY_SIZE(calibrate.phase_sum); i++)
			{
				const float phase_rad = phase_deg[i] * DEG_TO_RAD;
				calibrate.phase_sum[i].real += cosf(phase_rad);
				calibrate.phase_sum[i].imag += sinf(phase_rad);
			}

			// delay saving the settings
			save_settings_timer = SAVE_SETTINGS_MS;

			if (++calibrate.count >= CALIBRATE_COUNT)
			{	// finished summing, save the average

				const unsigned int index = (measurement_Hz <= 300) ? 0 : 1;   // 100Hz/1kHz

				// magnitudes
				for (unsigned int i = 0; i < ARRAY_SIZE(calibrate.mag_sum); i++)
					settings.open_probe_calibration[index].mag_rms[i] = calibrate.mag_sum[i] / calibrate.count;

				// phases
				for (unsigned int i = 0; i < ARRAY_SIZE(calibrate.phase_sum); i++)
					settings.open_probe_calibration[index].phase_deg[i] = (calibrate.phase_sum[i].real != 0) ? atan2f(calibrate.phase_sum[i].imag, calibrate.phase_sum[i].real) * RAD_TO_DEG : NAN;

				// set flag to say "open calibration done"
				settings.open_probe_calibration[index].done = 1;

				if (index == 0)
				{	// do the same again but at the next measurement frequency

					memset((void *)&calibrate, 0, sizeof(calibrate));
					set_measurement_frequency(1000);
				}
				else
				{	// done
					printf(NEWLINE "open probe calibration done" NEWLINE);

					// restore original measurement frequency
					set_measurement_frequency(settings.measurement_Hz);

					// back to normal measurement mode
					op_mode = OP_MODE_MEASURING;
				}

				draw_screen();
			}

			break;
		}

		case OP_MODE_SHORTED_PROBE_CALIBRATION:   // doing a SHORTED probe calibration
		{
			// sum a number of magnitudes
			for (unsigned int i = 0; i < ARRAY_SIZE(calibrate.mag_sum); i++)
				calibrate.mag_sum[i] += mag_rms[i];

			// sum a number of phases
			for (unsigned int i = 0; i < ARRAY_SIZE(calibrate.phase_sum); i++)
			{
				const float phase_rad = phase_deg[i] * DEG_TO_RAD;
				calibrate.phase_sum[i].real += cosf(phase_rad);
				calibrate.phase_sum[i].imag += sinf(phase_rad);
			}

			// delay saving the settings
			save_settings_timer = SAVE_SETTINGS_MS;

			if (++calibrate.count >= CALIBRATE_COUNT)
			{	// finished summing, save the average

				const unsigned int index = (measurement_Hz <= 300) ? 0 : 1;   // 100Hz/1kHz

				// magnitudes
				for (unsigned int i = 0; i < ARRAY_SIZE(calibrate.mag_sum); i++)
					settings.shorted_probe_calibration[index].mag_rms[i] = calibrate.mag_sum[i] / calibrate.count;

				// phases
				for (unsigned int i = 0; i < ARRAY_SIZE(calibrate.phase_sum); i++)
					settings.shorted_probe_calibration[index].phase_deg[i] = (calibrate.phase_sum[i].real != 0) ? atan2f(calibrate.phase_sum[i].imag, calibrate.phase_sum[i].real) * RAD_TO_DEG : NAN;

				// set flag to say "shorted calibration done"
				settings.shorted_probe_calibration[index].done = 1;

				if (index == 0)
				{	// do the same again but at the next measurement frequency

					memset((void *)&calibrate, 0, sizeof(calibrate));
					set_measurement_frequency(1000);
				}
				else
				{	// done
					printf(NEWLINE "shorted probe calibration done" NEWLINE);

					// restore original measurement frequency
					set_measurement_frequency(settings.measurement_Hz);

					// back to normal measurement mode
					op_mode = OP_MODE_MEASURING;
				}

				draw_screen();
			}

			break;
		}
	}
}

// ***********************************************************

int main(void)
{
	LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_AFIO);
	LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);

	LL_GPIO_AF_Remap_SWJ_NOJTAG();

	{	// stop stuff when debugging line-by-line
		DBGMCU->CR |= LL_DBGMCU_APB1_GRP1_IWDG_STOP;
		DBGMCU->CR |= LL_DBGMCU_APB1_GRP1_WWDG_STOP;
		DBGMCU->CR |= LL_DBGMCU_APB2_GRP1_TIM1_STOP;
		DBGMCU->CR |= LL_DBGMCU_APB1_GRP1_TIM2_STOP;
		DBGMCU->CR |= LL_DBGMCU_APB1_GRP1_TIM3_STOP;
		DBGMCU->CR |= LL_DBGMCU_APB1_GRP1_TIM4_STOP;
		DBGMCU->CR |= LL_DBGMCU_APB1_GRP1_I2C1_STOP;
		DBGMCU->CR |= LL_DBGMCU_APB1_GRP1_I2C2_STOP;
		DBGMCU->CR |= LL_DBGMCU_APB1_GRP1_CAN1_STOP;
	}

	NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);

	{	// get reset cause
		#pragma GCC diagnostic push
		#pragma GCC diagnostic ignored "-Wunused-variable"
		// -Wunused -Wunused-function -Wunused-label -Wunused-parameter -Wunused-value -Wunused-variable -Wunused-but-set-parameter -Wunused-but-set-variable
		reset_cause.por    = LL_RCC_IsActiveFlag_PORRST();
		reset_cause.pin    = LL_RCC_IsActiveFlag_PINRST();
		reset_cause.sft    = LL_RCC_IsActiveFlag_SFTRST();
		reset_cause.iwdg   = LL_RCC_IsActiveFlag_IWDGRST();
		reset_cause.wwdg   = LL_RCC_IsActiveFlag_WWDGRST();
		reset_cause.lpwr   = LL_RCC_IsActiveFlag_LPWRRST();
		reset_cause.lsirdy = LL_RCC_IsActiveFlag_LSIRDY();
		LL_RCC_ClearResetFlags();                           // cleart the flags ready for next reboot
		#pragma GCC diagnostic pop
	}

	#if 0
	{	// disable printf(), fread(), fwrite(), sscanf() etc buffering
		setbuf( stdin,  NULL);
		setbuf( stdout, NULL);
		setbuf( stderr, NULL);
		setvbuf(stdout, NULL, _IONBF, 0);
		setvbuf(stderr, NULL, _IONBF, 0);
	}
	#endif

	{
		button[BUTTON_HOLD].gpio_port = BUTT_HOLD_GPIO_Port;
		button[BUTTON_HOLD].gpio_pin  = BUTT_HOLD_Pin;

		button[BUTTON_SP].gpio_port   = BUTT_SP_GPIO_Port;
		button[BUTTON_SP].gpio_pin    = BUTT_SP_Pin;

		button[BUTTON_RCL].gpio_port  = BUTT_RCL_GPIO_Port;
		button[BUTTON_RCL].gpio_pin   = BUTT_RCL_Pin;
	}

	{	// set defaults
		settings.series_ohms    = SERIES_RESISTOR_OHMS;      // this can be calibrated using a DUT with a known resistance value
		settings.baudrate       = UART_BAUDRATE;
		settings.measurement_Hz = 1000;
//		settings.lcr_mode       = LCR_MODE_INDUCTANCE;
		settings.lcr_mode       = LCR_MODE_CAPACITANCE;
//		settings.lcr_mode       = LCR_MODE_RESISTANCE;
//		settings.flags          = 0;
		settings.flags         |= SETTING_FLAG_UART_DSO;     // send ADC data via the serial port
		settings.flags         |= SETTING_FLAG_SEND_BINARY;  // binary data
	}

	DWT_Delay_Init();
	HAL_Init();
	SystemClock_Config();
	#ifdef USE_IWDG
		MX_IWDG_Init();
	#endif
	LL_RCC_GetSystemClocksFreq(&rcc_clocks);
	MX_GPIO_Init();
	MX_USART1_UART_Init();

	{	// setup the goertzel filter
		const float normalized_freq = 2.0f / ADC_DATA_LENGTH;  // 2 cycles spanning the sample buffer
		goertzel_init(&goertzel, normalized_freq);
	}

	// fetch saved settings from flash
	read_settings();

	settings.baudrate = (settings.baudrate < 115200) ? 115200 : (settings.baudrate > 921600) ? 921600 : settings.baudrate;

	inv_series_ohms = 1.0f / settings.series_ohms;

	system_data.vi_measure_mode = vi_measure_mode_table[vi_measure_index];
	set_measure_mode_pins(system_data.vi_measure_mode);

	set_measurement_frequency(settings.measurement_Hz);

	screen_init();
	bootup_screen();

	printf(NEWLINE NEWLINE "rebooted M181 LCR Meter v%0.2f %s %s %s %s %s %s %s" NEWLINE,
		FW_VERSION,
		reset_cause.por    ? "POR"    : "por",
		reset_cause.pin    ? "PIN"    : "pin",
		reset_cause.sft    ? "SFT"    : "sft",
		reset_cause.iwdg   ? "IWDG"   : "iwdg",
		reset_cause.wwdg   ? "WWDG"   : "wwdg",
		reset_cause.lpwr   ? "LPWR"   : "lpwr",
		reset_cause.lsirdy ? "LSIRDY" : "lsirdy");

	// *************************************

	{	// wait until the user has released all buttons (for at least 1000ms)

		uint32_t butt_tick = sys_tick;

		while (1)
		{
			__WFI();

			const uint32_t tick = sys_tick;

			for (unsigned int i = 0; i < ARRAY_SIZE(button); i++)
				if (button[i].debounce > 0)
					butt_tick = tick;

			if ((tick - butt_tick) >= 1000)
				break;

			#ifdef USE_IWDG
				// feed the dog
				service_IWDG(0);
			#endif
		}

		// clear all butt press info
		for (unsigned int i = 0; i < ARRAY_SIZE(button); i++)
		{
			button[i].processed  = 0;
			button[i].debounce   = 0;
			button[i].pressed_ms = 0;
			button[i].released   = 0;
			button[i].held_ms    = 0;
		}
	}

	// *************************************

	MX_ADC_Init();
	MX_TIM3_Init();

	// give the user more time to read the bootup screen
	LL_mDelay(1000);

	// start making use of the incoming ADC blocks
	initialising = 0;

	#ifdef USE_IWDG
		// feed the dog
		service_IWDG(1);
	#endif

	while (1)
	{
		// nothing's going to change until some interrupt or other does something useful
		// so may as well wait here until the next interrupt occurs (timer, adc sampling, uart etc)
		//
		// some people put the CPU into sleep mode here to save battery energy (an interrupt will wake us up)
		//
		__DSB();    // sweep up before having a little dose
		__WFI();    // wait here until next interrupt occurs
//		__WFE();    // wait here in low power mode until next interrupt occurs

		// servicing any and all user input without delay is the highest priority
		process_buttons();

		// process any data received via the serial port
		process_uart_receive();

		// process the new ADC sample block
		process_ADC_exec();

//		const unsigned int prev_vi_measure_mode = system_data.vi_measure_mode;
		system_data.vi_measure_mode = vi_measure_mode_table[vi_measure_index];

		if (vi_measure_index >= VI_MODE_DONE)
		{	// completed another full measurement cycle

			process_data();
			process_op_mode();
			draw_screen();
			send_data();

			frames++;

			vi_measure_index = 0;         // start next data capture
		}
		//else
		//if (system_data.vi_measure_mode != prev_vi_measure_mode && draw_screen_count > 0)
		//	draw_measurement_mode();

		// save any unsaved settings to flash (we don't have an EEPROM etc)
		if (save_settings_timer == 0)
		{
			if (write_settings() < 0)     // save settings to flash
				write_settings();         // failed, have a 2nd go
			save_settings_timer = -1;     // don't try saving again (until the next time)

			printf(NEWLINE "settings saved" NEWLINE);
		}

		#ifdef USE_IWDG
			service_IWDG(0);
		#endif
	}
}

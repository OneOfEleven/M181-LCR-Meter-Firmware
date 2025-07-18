/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 * Developed by: Jaishankar M
 * Re-worked by: OneOfEleven
 ******************************************************************************
 */

#include <string.h>
#include <stdio.h>
#include <float.h>
#include <limits.h>

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
	GPIO_TypeDef     *gpio_port;
	uint16_t          gpio_pin;
	int16_t           debounce;        // counter for debounce
	volatile uint32_t pressed_tick;    // tick value when button was pressed
	volatile uint8_t  released;        // true when button is released
	volatile uint32_t held_ms;         // number of ms the button is/was held for
	uint8_t           processed;       // set once we've delt with this button
} t_button;

typedef struct {
	int16_t adc;
	int16_t afc;
} t_adc_dma_data_16;

typedef struct {
	int32_t adc;
	int32_t afc;
} t_adc_dma_data_32;

typedef struct t_comp {
	float re;
	float im;

	t_comp()
	{
		re = 0.0f;
		im = 0.0f;
	}

	t_comp(const float _re, const float _im)
	{
		re = _re;
		im = _im;
	}
} t_comp;

#pragma pack(push, 1)
typedef struct {
	union {
		uint32_t marker;
		uint8_t  marker_b[sizeof(uint32_t)];
	};
	union {
		uint16_t crc;
		uint8_t  crc_b[sizeof(uint16_t)];
	};
	union {
		float    data[ADC_DATA_LENGTH * 8];
		uint8_t  data_b[sizeof(float) * ADC_DATA_LENGTH * 8];
	};
} t_packet;
#pragma pack(pop)
/*
static const uint16_t omega_7x10[] = {
	0b0011100,       // 1
	0b0100010,       // 2
	0b1000001,       // 3
	0b1000001,       // 4
	0b1000001,       // 5
	0b1000001,       // 6
	0b0100010,       // 7
	0b0011100,       // 8
	0b0010100,       // 9
	0b0100010        // 10
};
*/
static const uint16_t omega_11x18[] = {
	0b00000000000,   // 1
	0b00000000000,   // 2
	0b00000000000,   // 3
	0b00000000000,   // 4
	0b00000000000,   // 5
	0b00001111000,   // 6
	0b00011111100,   // 7
	0b00111001110,   // 8
	0b01100000011,   // 9
	0b01100000011,   // 10
	0b01100000011,   // 11
	0b01100000011,   // 12
	0b00110000110,   // 13
	0b00011001100,   // 14
	0b00011001100,   // 15
	0b00011001100,   // 16
	0b01111001111,   // 17
	0b01111001111    // 18
};

#if defined(USE_IWDG) && defined(HAL_IWDG_MODULE_ENABLED)
	IWDG_HandleTypeDef hiwdg          = {0};
#endif
UART_HandleTypeDef     huart1         = {0};
DMA_HandleTypeDef      hdma_usart1_tx = {0};
TIM_HandleTypeDef      htim3          = {0};
ADC_HandleTypeDef      hadc1          = {0};
#ifdef DUAL_ADC_MODE
	ADC_HandleTypeDef  hadc2          = {0};
#endif
DMA_HandleTypeDef      hdma_adc1      = {0};

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
	volatile uint32_t iwdg_tick = 0;
#endif

uint32_t              draw_screen_count  = 0;
char                  buffer_display[26] = {0};

t_button              button[BUTTON_NUM] = {0};

const uint16_t        DAC_resolution                  = 256;  // 8-bit
volatile unsigned int sine_table_index                = 0;    //
uint8_t               sine_table[ADC_DATA_LENGTH / 2] = {0};  // length is matched with the ADC sampling (length = one sine cycle)

uint16_t              measurement_Hz        = 1000;
float                 measurement_amplitude = 1.0;            // 0.0 = 0%, 1.0 = 100%, -1.0 = 100% phase inverted

unsigned int          op_mode = OP_MODE_MEASURING;

unsigned int          display_hold = 0;                       // '1' to pause the display

// for the calibration modes
struct {
	uint16_t          Hz;
	int               count;
	float             mag_sum[8];
	t_comp            phase_sum[8];
} calibrate = {0};

unsigned int          volt_gain_sel   = 0;
unsigned int          amp_gain_sel    = 0;
unsigned int          gain_changed    = 0;

float                 high_gain       = 101;
float                 inv_series_ohms = 1.0f / SERIES_RESISTOR_OHMS;

// ADC DMA sample buffer
t_adc_dma_data_16     adc_dma_buffer[2][ADC_DATA_LENGTH];      // *2 for DMA double buffering (ADC/DMA is continuously running)

uint32_t              frames  = 0;        // just a frame counter

// ADC sample block averaging buffer
// we take several sample blocks and average them together to reduce noise/increase dynamic range
t_adc_dma_data_32     adc_buffer_sum[ADC_DATA_LENGTH] = {0};                        // summing buffer
unsigned int          adc_buffer_sum_count            = 0;                          // number of sums so far done

#pragma pack(push, 1)
float                 adc_data[8][ADC_DATA_LENGTH] = {0};
#pragma pack(pop)

// non-zero if waveform clipping/saturation is detected (per block)
// we use the histogram method to detect clipped/saturate samples
uint8_t               adc_data_clipping[4] = {0};
uint8_t               adc_data_clipped[4]  = {0};

float                 mag_rms[8]   = {0};
float                 phase_deg[8] = {0};

// custom HW VI mode sequence order to minimize mode switching time
// because of a HW design floor (takes time for HW to settle after changing the HW GS/VI mode pins)
const unsigned int    vi_measure_mode_table[] = {0, 1, 3, 2};
volatile unsigned int vi_measure_index = 0;
unsigned int          prev_vi_mode     = -1;

// system settings
volatile int          save_settings_timer = -1;  // reduce flash writes (reduces wear on the flash)
t_settings            settings            = {0};

t_system_data         system_data = {0};

// temp buffer - used for the Goerttzel output samples
t_comp                tmp_buf[ADC_DATA_LENGTH];

// for TX'ing binary packets vis the serial port
t_packet              tx_packet;

// ***********************************************************

//__STATIC_INLINE void DAC_write(const uint8_t dat)
__STATIC_FORCEINLINE void DAC_write(const uint8_t dat)
{
	GPIOB->ODR = (GPIOB->ODR & 0xFFFFFF00) | dat;
}

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
			HAL_GPIO_WritePin(VI_pin_GPIO_Port, VI_Pin, LOW);
			HAL_GPIO_WritePin(GS_pin_GPIO_Port, GS_Pin, HIGH);
			break;

		case VI_MODE_AMP_LO_GAIN:            // low gain current mode
			HAL_GPIO_WritePin(VI_pin_GPIO_Port, VI_Pin, HIGH);
			HAL_GPIO_WritePin(GS_pin_GPIO_Port, GS_Pin, HIGH);
			break;

		case VI_MODE_VOLT_HI_GAIN:           // high gain voltage mode
			HAL_GPIO_WritePin(VI_pin_GPIO_Port, VI_Pin, LOW);
			HAL_GPIO_WritePin(GS_pin_GPIO_Port, GS_Pin, LOW);
			break;

		case VI_MODE_AMP_HI_GAIN:            // high gain current mode
			HAL_GPIO_WritePin(VI_pin_GPIO_Port, VI_Pin, HIGH);
			HAL_GPIO_WritePin(GS_pin_GPIO_Port, GS_Pin, LOW);
			break;
	}
}

void reboot(void)
{
	__disable_irq();
	HAL_GPIO_WritePin(LED_pin_GPIO_Port, LED_Pin, GPIO_PIN_SET);	// LED on
	HAL_NVIC_SystemReset();
	while (1) {}
}

#ifdef USE_IWDG
	void MX_IWDG_Init(void)
	{
		memset(&hiwdg, 0, sizeof(hiwdg));

		hiwdg.Instance = IWDG;

		// Reload value for 8 second IWDG TimeOut
		// So Set Reload Counter Value = (LSI_VALUE * 8) / 256
		hiwdg.Init.Prescaler = IWDG_PRESCALER_256;	// 4, 8, 16, 32, 64, 128 or 256
		hiwdg.Init.Reload    = (LSI_VALUE * 8) / 256;
		if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
		{
			hiwdg.Instance = NULL;
			return;
		}

		iwdg_tick = 0;
		HAL_IWDG_Refresh(&hiwdg);
	}

	void service_IWDG(const uint8_t force_update)
	{	// service the watchdog
		if (hiwdg.Instance != NULL && (iwdg_tick >= 4000 || force_update))
		{
			iwdg_tick = 0;
			HAL_IWDG_Refresh(&hiwdg);
		}
	}
#endif

void start_ADC(void)
{
	frames = 0;

	// non-stop ADC double buffered sampling
	#ifdef DUAL_ADC_MODE
		HAL_ADC_Start(&hadc2);
		HAL_ADCEx_MultiModeStart_DMA(&hadc1, (uint32_t *)adc_dma_buffer, ADC_DATA_LENGTH * 2);
	#else
		HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_dma_buffer, ADC_DATA_LENGTH * 2);
	#endif

//	sine_table_index = ARRAY_SIZE(sine_table) * 0.285; // align the sinewave (@1kHz)

	HAL_TIM_Base_Start_IT(&htim3);
}

void stop_ADC(void)
{
	HAL_TIM_Base_Stop(&htim3);

	#ifdef DUAL_ADC_MODE
		HAL_ADCEx_MultiModeStop_DMA(&hadc1);
		HAL_ADC_Stop(&hadc2);
	#else
		HAL_ADC_Stop_DMA(&hadc1);
	#endif
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
	float re;
	float im;
} t_goertzel;

t_goertzel goertzel = {0};

// feed the supplied samples through the Goertzel filter
//
void goertzel_block(const float *samples, const unsigned int len, t_goertzel *g)
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
	g->re = re * scale;
	g->im = im * scale;
}

int goertzel_wrap(const float *in_samples, t_comp *out_samples, const unsigned int len, const unsigned int g_len, t_goertzel *g)
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

		const float re = (m1 * g->cos) - m2;
		const float im = -m1 * g->sin;

		// correct the output sample amplitude
		out_samples[k].re = re * scale;
		out_samples[k].im = im * scale;
	
		// exit if a button is pressed
		//
		// servicing user input is paramount
		//
		for (unsigned int b = 0; b < BUTTON_NUM; b++)
			if (button[b].pressed_tick > 0 || button[b].released)
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

	g->re    = 0;
	g->im    = 0;
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

void set_measurement_frequency(const uint32_t Hz)
{
	// TODO: make the amplitude adjustment automatic by inspecting the sine wave using a histogram of the sampled samples
	//       spikes in the histogram indicate clipping (several similar values)

	if (Hz == 100)
	{
		measurement_Hz        = 100;
		measurement_amplitude = 0.52;
	}
	else
	if (Hz == 300)
	{
		measurement_Hz        = 300;
		measurement_amplitude = 0.58;
	}
	else
	{	// default to 1kHz
		measurement_Hz        = 1000;
		measurement_amplitude = 1.0;
	}

	{	// fill the sine wave look-up table with one complete sine cycle
		const float scale      = (DAC_resolution - 1) * measurement_amplitude * 0.5f;
		const float phase_step = (float)(2.0 * M_PI) / ARRAY_SIZE(sine_table);
		for (unsigned int i = 0; i < ARRAY_SIZE(sine_table); i++)
			sine_table[i] = (uint8_t)floorf(((1.0f + sinf(phase_step * i)) * scale) + 0.5f); // raised sine
	}

	if (measurement_Hz > 0)
	{	// set the timer rate
		const uint32_t timer_rate_Hz = (ADC_DATA_LENGTH / 2) * measurement_Hz;
		const uint32_t period        = (((HAL_RCC_GetHCLKFreq() / (htim3.Init.Prescaler + 1)) + (timer_rate_Hz / 2)) / timer_rate_Hz) - 1;
		__HAL_TIM_SET_AUTORELOAD(&htim3, period);
	}
}

#if 1
	float phase_diff(const t_comp c1, const t_comp c2)
	{
		// conj multiply
		const t_comp d = t_comp((c1.re * c2.re) + (c1.im * c2.im), (c1.re * c2.im) - (c1.im * c2.re));

		// phase
		return (d.re != 0.0f) ? atan2f(d.im, d.re) * RAD_TO_DEG : NAN;
	}

	float phase_diff(const float phase_deg_1, const float phase_deg_2)
	{
		const float phi1 = phase_deg_1 * DEG_TO_RAD;
		const float phi2 = phase_deg_2 * DEG_TO_RAD;
		return phase_diff(t_comp(cosf(phi1), sinf(phi1)), t_comp(cosf(phi2), sinf(phi2)));
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

	for (unsigned int buf_index = 0; buf_index < ARRAY_SIZE(adc_data); buf_index++)
	{
		const unsigned int vi_mode = buf_index >> 1;

		#if !defined(GOERTZEL_FILTER_LENGTH) || (GOERTZEL_FILTER_LENGTH <= 0)
			uint8_t filter = 0;	// don't Goertzel filter
		#else
			uint8_t filter = 1; // do Goertzel filter
		#endif

		uint8_t clipped = 0;

		if (op_mode == OP_MODE_MEASURING)
		{	// don't filter if we're not going to use the data
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
			//    remove waveform DC offset
			//   compute waveform RMS magnitude
			//   compute waveform phase

			register float *buf = adc_data[buf_index];   // point to the ADC samples

			{	// compute waveform RMS magnitude
				register float sum = 0;
				for (unsigned int k = 0; k < ADC_DATA_LENGTH; k++)
					sum += SQR(buf[k]);
				sum *= 1.0f / ADC_DATA_LENGTH;
				mag_rms[buf_index] = sqrtf(sum);
			}

			{	// compute waveform phase
				goertzel_block(buf, ADC_DATA_LENGTH, &goertzel);
				phase_deg[buf_index] = (goertzel.re != 0.0f) ? fmodf((atan2f(goertzel.im, goertzel.re) * RAD_TO_DEG) + 270, 360) : NAN;
			}
		}
		else
		{	// use Goertzel DFT to filter the waveform

			register t_comp *buf = tmp_buf;           // point to Goertzel dft output buffer

			// do it
			if (goertzel_wrap(adc_data[buf_index], buf, ADC_DATA_LENGTH, GOERTZEL_FILTER_LENGTH, &goertzel) < 0)
				return -1;

			{	// compute RMS magnitude and save the Goertzel filtered output samples
				register float *buf_out = adc_data[buf_index];
				register float sum = 0;
				for (unsigned int k = 0; k < ADC_DATA_LENGTH; k++)
				{
					register const t_comp samp = buf[k];
					sum += SQR(samp.re) + SQR(samp.im);
					buf_out[k] = samp.re;             // replace the unfiltered sample with the Goertzel filtered sample
				}
				sum *= 1.0f / ADC_DATA_LENGTH;
				mag_rms[buf_index] = sqrtf(sum);
			}

			{	// compute waveform phase
				goertzel_block(adc_data[buf_index], ADC_DATA_LENGTH, &goertzel);
				phase_deg[buf_index] = (goertzel.re != 0.0f) ? fmodf((atan2f(goertzel.im, goertzel.re) * RAD_TO_DEG) + 270, 360) : NAN;
			}
		}
	}

	return 0;
}

void combine_afc(float *avg_rms, float *avg_deg)
{	// combine AFC mag/phase results (the AFC sample blocks are all the same so use the average)

	unsigned int sum_count = 0;
	float        sum_rms   = 0;
	t_comp       sum_phase = {0, 0};

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
		sum_phase.re += cosf(phase_rad);
		sum_phase.im += sinf(phase_rad);

		sum_count++;
	}

	*avg_rms = sum_rms / sum_count;
	*avg_deg = (sum_phase.re != 0.0f) ? atan2f(sum_phase.im, sum_phase.re) * RAD_TO_DEG : NAN;
}

#if defined(MEDIAN_SIZE) && (MEDIAN_SIZE >= 3)

	int compare_float(const void *a, const void *b)
	{
		return (*(float *)a - *(float *)b);
	}

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
			if (settings.open_probe_calibration->done)
			{	// apply open probe calibration

				const unsigned int freq_index = (calibrate.Hz == 100) ? 0 : 1;   // 100Hz/1kHz

				const float v_cal_rms_lo = settings.open_probe_calibration[freq_index].mag_rms[VI_MODE_VOLT_LO_GAIN * 2];
				const float i_cal_rms_hi = settings.open_probe_calibration[freq_index].mag_rms[VI_MODE_AMP_HI_GAIN  * 2] * inv_series_ohms * high_scale;

			 	const float imp_cal = v_cal_rms_lo / i_cal_rms_hi;

				system_data.impedance = (imp_cal * system_data.impedance) / (imp_cal - system_data.impedance);
			}

			if (settings.shorted_probe_calibration->done)
			{	// apply shorted probe calibration



				// TODO:



			}
		}
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

	// TODO: find out if the following 'fabsf()'s should be there or not
	//
	const float ser_resistive    = system_data.impedance * fabsf(cs);                // Rs
	const float ser_reactance    = system_data.impedance * fabsf(sn);                // Xs

	if (ser_resistive == 0 || ser_reactance == 0 || omega == 0)
		return;                                                                      // prevent div-by-0 errors

	const float ser_inductance   = ser_reactance / omega;                            // L = X / ω
	const float ser_capacitance  = 1.0f / (omega * ser_reactance);                   // C = 1 / (ωX)
	const float ser_esr          = ser_resistive;                                    // R
	const float ser_tan_delta    = ser_resistive / ser_reactance;                    // D = R / X

	// these are each the same result, just done in a different way
//	const float ser_qf_ind       = (omega * ser_inductance) / ser_resistive;         // Q = (ωL) / R
//	const float ser_qf_cap       = 1.0f / (omega * ser_capacitance * ser_resistive); // Q = 1 / (ωCR)
	const float ser_qf_res       = ser_reactance / ser_resistive;                    // Q = X / R or 1 / D

	// series to parallel
	const float p                = SQR(ser_resistive) + SQR(ser_reactance);
	const float par_resistive    = p / ser_resistive;
	const float par_reactance    = p / ser_reactance;

	const float par_inductance   = par_reactance / omega;                            // L = X / ω
	const float par_capacitance  = 1.0f / (omega * par_reactance);                   // C = 1 / (ωX)
	const float par_esr          = par_resistive;                                    // R
	const float par_tan_delta    = par_resistive / par_reactance;                    // D = R / X

	// these are each the same result, just done in a different way
//	const float par_qf_ind       = (omega * par_inductance) / par_resistive;         // Q = (ωL) / R
//	const float par_qf_cap       = 1.0f / (omega * par_capacitance * par_resistive); // Q = 1 / (ωCR)
	const float par_qf_res       = par_reactance / par_resistive;                    // Q = X / R or 1 / D

	system_data.series.inductance    = ser_inductance;
	system_data.series.capacitance   = ser_capacitance;
	system_data.series.resistance    = system_data.impedance;
	system_data.series.esr           = ser_esr;
	system_data.series.tan_delta     = ser_tan_delta;
//	system_data.series.qf            = ser_qf_ind;
//	system_data.series.qf            = ser_qf_cap;
	system_data.series.qf            = ser_qf_res;

	system_data.parallel.inductance  = par_inductance;
	system_data.parallel.capacitance = par_capacitance;
	system_data.parallel.resistance  = system_data.impedance;
	system_data.parallel.esr         = par_esr;
	system_data.parallel.tan_delta   = par_tan_delta;
//	system_data.parallel.qf          = par_qf_ind;
//	system_data.parallel.qf          = par_qf_cap;
	system_data.parallel.qf          = par_qf_res;

	// **************************
}

void process_ADC(const void *buffer)
{
	// process the new ADC 12-bit samples

//	HAL_GPIO_WritePin(TP21_pin_GPIO_Port, TP21_Pin, HIGH);   // TEST

	// point to the new block of ADC samples
	const t_adc_dma_data_16 *adc_buffer = (t_adc_dma_data_16 *)buffer;

	// current VI mode index
	unsigned int vi_index = vi_measure_index;

	// ignore this sample block if we've completed the VI measurement scan
	if (vi_index >= VI_MODE_DONE)
		return;

	// use a table to set HD mode pins in a custom order to cope with a HW design floor
	const unsigned int vi_mode = vi_measure_mode_table[vi_index];

	const unsigned int buf_index = vi_mode * 2;

	if (vi_index == 0 && adc_buffer_sum_count == 0)
	{	// the first sample block after setting the HW mode pins

		set_measure_mode_pins(vi_mode);                                     // ensure the HW mode pins are set correctly

		// reset the clipping detected flags
		memset(adc_data_clipping, 0, sizeof(adc_data_clipping));

		HAL_GPIO_WritePin(LED_pin_GPIO_Port, LED_Pin, GPIO_PIN_SET);        // TEST only, LED on
	}

	// each time the HW VI mode is changed, the ADC input sees a large unwanted spike/DC-offset that takes time to settle :(
	// so we simply discard a number of sample blocks after each HW VI mode change
	//
	// actually, the spike only appears to occur after the GS pin (gain setting) is changed

	// skip less blocks if the gain pin hasn't changed
//	const unsigned int skip_block_count = ((vi_mode >> 1) == (prev_vi_mode >> 1)) ? MODE_SWITCH_BLOCK_WAIT_SHORT : MODE_SWITCH_BLOCK_WAIT_LONG;
	const unsigned int skip_block_count = MODE_SWITCH_BLOCK_WAIT_LONG;

	if (adc_buffer_sum_count >= skip_block_count)
	{	// add the new sample block to the averaging buffer

		// invert the current (I) ADC waveform to counter-act the inverting OP-AMP stage
		register const int16_t adc_sign = (vi_mode == VI_MODE_AMP_LO_GAIN || vi_mode == VI_MODE_AMP_HI_GAIN) ? -1 : 1;
		register const int16_t afc_sign = 1;

		if (vi_mode >= VI_MODE_VOLT_HI_GAIN)
		{	// only bother doing a histogram for the HIGH gain modes

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

	// decide which modes are useful and which are not
	//
	// we drop the hi-gain blocks if they are clipping (the data would useless)
	// we drop the lo-gain blocks if the hi-gain blocks are usable (no clipping detected)
	//
	unsigned int average_count = SLOW_ADC_AVERAGE_COUNT;
	if (op_mode == OP_MODE_MEASURING)
	{
		if (display_hold)
			average_count = 8;                           // display is paused, we're doing nothing but sending the samples to the PC
		else
		if (adc_data_clipping[vi_mode])
			average_count = 1;                           // this block of samples are clipping, drop them, move on to the next mode
		else
		if (vi_mode < VI_MODE_VOLT_HI_GAIN && !adc_data_clipped[VI_MODE_VOLT_HI_GAIN + vi_mode])
			average_count = 1;                           // hi-gain samples were not clipped on the previous run, so drop these lo-gain samples
		else
		if (settings.flags & SETTING_FLAG_FAST_UPDATES)
			average_count = FAST_ADC_AVERAGE_COUNT;      // user has selected faster display updates
	}

	if (++adc_buffer_sum_count < (skip_block_count + average_count))
		return;                                          // not yet summed the desired number of sample blocks

	// we've now summed the desired number of sample blocks, they're now ready to be re-scaled, saved and DC offset removed for use later use ..

	// get ready for next measurement VI mode
	vi_index++;

	// set the GS/VI pins ready for the next measurement run
	set_measure_mode_pins(vi_measure_mode_table[vi_index]);

	{	// fetch & re-scale the summed ADC sample blocks to create an average (reduces noise)

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

			if (!adc_data_clipping[vi_mode] || op_mode != OP_MODE_MEASURING)   // don't bother if the samples are being clipped (the block will not be used)
//			if (!adc_data_clipping[vi_mode] && op_mode == OP_MODE_MEASURING)   // don't bother if the samples are being clipped (the block will not be used)
			{	// ADC input

				// compute the DC offset
				register float sum = 0;
				for (unsigned int i = 0; i < ADC_DATA_LENGTH; i++)
					sum += buf_adc[i];
				sum *= 1.0f / ADC_DATA_LENGTH;

				// LPF
				settings.input_offset.adc[vi_mode] = ((1.0f - coeff) * settings.input_offset.adc[vi_mode]) + (coeff * sum);

				if (!display_hold)                            // don't remove offset if display HOLD is active
				{	// subtract/remove the DC offset
					sum = settings.input_offset.adc[vi_mode];
					for (unsigned int i = 0; i < ADC_DATA_LENGTH; i++)
						buf_adc[i] -= sum;
				}
			}

			{	// AFC input

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

		HAL_GPIO_WritePin(LED_pin_GPIO_Port, LED_Pin, GPIO_PIN_RESET);           // TEST only, LED off
	}
}

// ***********************************************************

// available font sizes ..
//  Font_7x10
//  Font_11x18
//  Font_16x26
//
// Font_7x10  .. small general
// Font_11x18 .. big

#define DRAW_LINES          // if you want horizontal lines drawn

#define     OFFSET_X      0
#define     LINE_SPACING  13

#ifdef DRAW_LINES
	#define LINE2_Y      (LINE_SPACING + 5)
	#define LINE3_Y      (10 + LINE2_Y - 1 + LINE_SPACING)
#else
	#define LINE2_Y      (LINE_SPACING + 3)
	#define LINE3_Y      (10 + LINE2_Y + 1 + LINE_SPACING)
#endif

#define     LINE4_Y      (LINE3_Y + LINE_SPACING)

void print_sprint(const unsigned int digit, const float value, char *output_char, const unsigned int out_max_size)
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
			// TODO: fix
//	        if (v < 1e-3f)
//				snprintf(output_char, out_max_size, "%0.0fu", value * 1e6f); // 123u
//			else
//	        if (v < 1e0f)
//				snprintf(output_char, out_max_size, "%0.0fm", value * 1e3f); // 123m
//			else
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
			// TODO: fix
//	        if (v < 1e-3f)
//				snprintf(output_char, out_max_size, "%0.0fu", value * 1e6f); // 123u
//			else
//	        if (v < 1e0f)
//				snprintf(output_char, out_max_size, "%0.0fm", value * 1e3f); // 123m
//			else
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
        uint16_t rowData = symbol[row] << (16 - symbolWidth);

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

	HAL_Delay(300);

	ssd1306_Fill(Black);
	ssd1306_UpdateScreen();
}

void bootup_screen(void)
{
//	ssd1306_Fill(Black);

	ssd1306_SetCursor(0, 0);
	ssd1306_WriteString("M181", &Font_7x10, White);

	ssd1306_SetCursor(90, 0);
	sprintf(buffer_display, "v%.2f", FW_VERSION);
	ssd1306_WriteString(buffer_display, &Font_7x10, White);

	ssd1306_SetCursor(16, 14);
	ssd1306_WriteString("LCR Meter", &Font_11x18, White);

	// dotted line
	ssd1306_dotted_hline(10, SSD1306_WIDTH - 10, 3, 32 - 1, White);

	ssd1306_SetCursor(5, 38);
	ssd1306_WriteString("HW by JYETech", &Font_7x10, White);
	ssd1306_SetCursor(5, 52);
	ssd1306_WriteString("FW by Jai & 1o11", &Font_7x10, White);

	ssd1306_UpdateScreen();
}

void draw_measurement_mode(void)
{
	ssd1306_SetCursor(SSD1306_WIDTH - 1 - (1 * Font_7x10.width), 0);
	snprintf(buffer_display, sizeof(buffer_display), "%u", system_data.vi_measure_mode);
	ssd1306_WriteString(buffer_display, &Font_7x10, White);

	ssd1306_UpdateScreen();
}

void draw_screen(void)
{
	const uint8_t x1 = 20;
	const uint8_t x2 = 62;
//	const uint8_t x3 = 75;

	// clear the screen
	ssd1306_Fill(Black);

	const uint8_t par  = settings.flags & SETTING_FLAG_PARALLEL;

	switch (op_mode)
	{
		default:
		case OP_MODE_MEASURING:

			// ***************************
			// Line 1

			// serial/parallel mode
			ssd1306_SetCursor(OFFSET_X, 0);
			ssd1306_WriteString(par ? "PAR" : "SER", &Font_7x10, White);

			// measurement frequency
			ssd1306_SetCursor(7 * 5, 0);
			if (measurement_Hz < 1000)
				snprintf(buffer_display, sizeof(buffer_display), "%3u Hz", measurement_Hz);
			else
				snprintf(buffer_display, sizeof(buffer_display), "%2u kHz", measurement_Hz / 1000);
			ssd1306_WriteString(buffer_display, &Font_7x10, White);

			if (display_hold)
			{
				ssd1306_SetCursor(SSD1306_WIDTH - 1 - (4 * Font_7x10.width), 0);
				ssd1306_WriteString("HOLD", &Font_7x10, White);
			}
			else
			if (settings.flags & SETTING_FLAG_FAST_UPDATES)
			{
				ssd1306_SetCursor(SSD1306_WIDTH - 1 - (4 * Font_7x10.width), 0);
				ssd1306_WriteString("fast", &Font_7x10, White);
			}

			#if 0
				// VI phase
				ssd1306_SetCursor(SSD1306_WIDTH - 1 - (5 * 7), 0);
				print_sprint(4, system_data.vi_phase_deg, buffer_display, sizeof(buffer_display));
				ssd1306_WriteString(buffer_display, &Font_7x10, White);
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
						snprintf(buffer_display, sizeof(buffer_display), "L");
						break;

					case LCR_MODE_CAPACITANCE:
						value = par ? system_data.parallel.capacitance : system_data.series.capacitance;
						snprintf(buffer_display, sizeof(buffer_display), "C");
						break;

					case LCR_MODE_RESISTANCE:
						value = par ? system_data.parallel.resistance : system_data.series.resistance;
						snprintf(buffer_display, sizeof(buffer_display), "R");
						break;

					case LCR_MODE_AUTO:
						// TODO:
						break;
				}

				ssd1306_SetCursor(OFFSET_X, LINE2_Y);
				ssd1306_WriteString(buffer_display, &Font_11x18, White);

				ssd1306_MoveCursor(10, 0);    // move right

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
							snprintf(buffer_display, sizeof(buffer_display), "%0.1f", value);
						else
							print_sprint(4, value, buffer_display, sizeof(buffer_display));
						break;

					case LCR_MODE_CAPACITANCE:
						if (unit == 'p')
							snprintf(buffer_display, sizeof(buffer_display), "%0.1f", value);
						else
							print_sprint(4, value, buffer_display, sizeof(buffer_display));
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
							snprintf(buffer_display, sizeof(buffer_display), "%0.1f", value);
							//snprintf(buffer_display, sizeof(buffer_display), "%d", (int)value);
						else
							print_sprint(4, value, buffer_display, sizeof(buffer_display));
						break;

					case LCR_MODE_AUTO:
						break;
				}

				ssd1306_WriteString(buffer_display, &Font_11x18, White);

				ssd1306_MoveCursor(6, 0);    // move right

				switch (settings.lcr_mode)
				{
					case LCR_MODE_INDUCTANCE:
					{
						unsigned int i = 0;
						if (unit != ' ')
							buffer_display[i++] = unit;
						buffer_display[i++] = 'H';
						buffer_display[i++] = '\0';
						ssd1306_WriteString(buffer_display, &Font_11x18, White);
						break;
					}

					case LCR_MODE_CAPACITANCE:
					{
						unsigned int i = 0;
						if (unit != ' ')
							buffer_display[i++] = unit;
						buffer_display[i++] = 'F';
						buffer_display[i++] = '\0';
						ssd1306_WriteString(buffer_display, &Font_11x18, White);
						break;
					}

					case LCR_MODE_RESISTANCE:
					{
						if (unit != ' ')
						{
							buffer_display[0] = unit;
							buffer_display[1] = '\0';
							ssd1306_WriteString(buffer_display, &Font_11x18, White);
						}
						else
							ssd1306_MoveCursor(4, 0);    // move right

						uint16_t x;
						uint16_t y;
						ssd1306_GetCursor(&x, &y);
						print_custom_symbol(x + 2, LINE2_Y - 3, omega_11x18, 11, 18);

						break;
					}

					case LCR_MODE_AUTO:
						break;
				}
			}

			#if 1
			{	// show gain setting for V and I modes
				buffer_display[0] = volt_gain_sel ? 'H' : 'L';
				buffer_display[1] = amp_gain_sel  ? 'H' : 'L';
				buffer_display[2] ='\0';
				ssd1306_SetCursor(SSD1306_WIDTH - 1 - (2 * Font_7x10.width), LINE2_Y + 5);
				ssd1306_WriteString(buffer_display, &Font_7x10, White);
			}
			#endif

			// ***************************
			// Line 3

			# if 1

				{	// voltage
					float value = (system_data.rms_voltage_adc >= 0) ? system_data.rms_voltage_adc : 0;
					value = adc_to_volts(value);
					const char unit = unit_conversion(&value);

					ssd1306_SetCursor(OFFSET_X, LINE3_Y);
					print_sprint(4, value, buffer_display, sizeof(buffer_display));
					unsigned int i = strlen(buffer_display);
					if (unit != ' ')
						buffer_display[i++] = unit;
					buffer_display[i++] = 'V';
					buffer_display[i++] = '\0';
					ssd1306_WriteString(buffer_display, &Font_7x10, White);
				}

				{	// current
					float value = (system_data.rms_current_adc >= 0) ? system_data.rms_current_adc : 0;
					value = adc_to_volts(value);
					const char unit = unit_conversion(&value);

					ssd1306_SetCursor(62, LINE3_Y);
					print_sprint(4, value, buffer_display, sizeof(buffer_display));
					unsigned int i = strlen(buffer_display);
					if (unit != ' ')
						buffer_display[i++] = unit;
					buffer_display[i++] = 'A';
					buffer_display[i++] = '\0';
					ssd1306_WriteString(buffer_display, &Font_7x10, White);
				}

			#else
			{	// Quality factor

				float value = par ? system_data.parallel.qf : system_data.series.qf;
				const char unit = unit_conversion(&value);

				ssd1306_SetCursor(OFFSET_X, LINE3_Y);
				ssd1306_WriteString("Q  ", &Font_7x10, White);

				ssd1306_SetCursor(x1, LINE3_Y);
				print_sprint(4, value, buffer_display, sizeof(buffer_display));
				unsigned int i = strlen(buffer_display);
				buffer_display[i++] = unit;
				buffer_display[i++] = '\0';
				ssd1306_WriteString(buffer_display, &Font_7x10, White);
			}
			#endif

			// ***************************
			// Line 4

			switch (settings.lcr_mode)
			{
				case LCR_MODE_INDUCTANCE:
				case LCR_MODE_CAPACITANCE:

					{	// ESR

						float value = par ? system_data.parallel.esr : system_data.series.esr;
						const char unit = unit_conversion(&value);

						ssd1306_SetCursor(OFFSET_X, LINE4_Y);
						ssd1306_WriteString("ER", &Font_7x10, White);

						ssd1306_SetCursor(x1, LINE4_Y);

						print_sprint(3, value, buffer_display, sizeof(buffer_display));
						unsigned int i = strlen(buffer_display);
						buffer_display[i++] = unit;
						buffer_display[i++] = '\0';
						ssd1306_WriteString(buffer_display, &Font_7x10, White);
					}

					#if 0
					{	// Tan Delta

						float value = par ? system_data.parallel.tan_delta : system_data.series.tan_delta;
						const char unit = unit_conversion(&value);

						ssd1306_SetCursor(x2, LINE4_Y);
						ssd1306_WriteString("D", &Font_7x10, White);

						ssd1306_SetCursor(x3, LINE4_Y);

						print_sprint(3, value, buffer_display, sizeof(buffer_display));
						unsigned int i = strlen(buffer_display);
						buffer_display[i++] = unit;
						buffer_display[i++] = '\0';
						ssd1306_WriteString(buffer_display, &Font_7x10, White);
					}
					#else
					{	// Quality factor

						float value = par ? system_data.parallel.qf : system_data.series.qf;
						const char unit = unit_conversion(&value);

						ssd1306_SetCursor(x2, LINE4_Y);
						ssd1306_WriteString("Q ", &Font_7x10, White);

						//ssd1306_SetCursor(x3, LINE4_Y);
						print_sprint(3, value, buffer_display, sizeof(buffer_display));
						unsigned int i = strlen(buffer_display);
						buffer_display[i++] = unit;
						buffer_display[i++] = '\0';
						ssd1306_WriteString(buffer_display, &Font_7x10, White);
					}
					#endif

					break;

				case LCR_MODE_RESISTANCE:

					{	// inductance

						float value = par ? system_data.parallel.inductance : system_data.series.inductance;
						const char unit = unit_conversion(&value);

						ssd1306_SetCursor(OFFSET_X, LINE4_Y);

						print_sprint(4, value, buffer_display, sizeof(buffer_display));
						unsigned int i = strlen(buffer_display);
						if (unit != ' ')
							buffer_display[i++] = unit;
						buffer_display[i++] = 'H';
						buffer_display[i++] = '\0';
						ssd1306_WriteString(buffer_display, &Font_7x10, White);
					}

					{	// Quality factor

						float value = par ? system_data.parallel.qf : system_data.series.qf;
						const char unit = unit_conversion(&value);

						ssd1306_SetCursor(x2, LINE4_Y);
						ssd1306_WriteString("Q ", &Font_7x10, White);

						print_sprint(3, value, buffer_display, sizeof(buffer_display));
						unsigned int i = strlen(buffer_display);
						buffer_display[i++] = unit;
						buffer_display[i++] = '\0';
						ssd1306_WriteString(buffer_display, &Font_7x10, White);
					}

					break;

				case LCR_MODE_AUTO:
					break;
			}

			// ***************************

			break;

		case OP_MODE_OPEN_PROBE_CALIBRATION:
			snprintf(buffer_display, sizeof(buffer_display), "OPEN cal %d", CALIBRATE_COUNT - calibrate.count - 1);
			//ssd1306_SetCursor(OFFSET_X, LINE2_Y);
			ssd1306_SetCursor(OFFSET_X, 5);
			//ssd1306_WriteString(buffer_display, &Font_7x10, White);
			ssd1306_WriteString(buffer_display, &Font_11x18, White);

			if (calibrate.Hz < 1000)
				snprintf(buffer_display, sizeof(buffer_display), " %u Hz", calibrate.Hz);
			else
				snprintf(buffer_display, sizeof(buffer_display), " %u kHz", calibrate.Hz / 1000);
			//ssd1306_SetCursor(OFFSET_X, LINE2_Y + 14);
			//ssd1306_WriteString(buffer_display, &Font_7x10, White);
			ssd1306_SetCursor(OFFSET_X, LINE3_Y - 5);
			ssd1306_WriteString(buffer_display, &Font_11x18, White);

			break;

		case OP_MODE_SHORTED_PROBE_CALIBRATION:
			snprintf(buffer_display, sizeof(buffer_display), "SHORT cal %d", CALIBRATE_COUNT - calibrate.count - 1);
			//ssd1306_SetCursor(OFFSET_X, LINE2_Y);
			ssd1306_SetCursor(OFFSET_X, 5);
			//ssd1306_WriteString(buffer_display, &Font_7x10, White);
			ssd1306_WriteString(buffer_display, &Font_11x18, White);

			if (calibrate.Hz < 1000)
				snprintf(buffer_display, sizeof(buffer_display), " %u Hz", calibrate.Hz);
			else
				snprintf(buffer_display, sizeof(buffer_display), " %u kHz", calibrate.Hz / 1000);
			//ssd1306_SetCursor(OFFSET_X, LINE2_Y + 14);
			//ssd1306_WriteString(buffer_display, &Font_7x10, White);
			ssd1306_SetCursor(OFFSET_X, LINE3_Y - 5);
			ssd1306_WriteString(buffer_display, &Font_11x18, White);

			break;
	}

	// ***************************
	// Line 4: UART mode

	ssd1306_SetCursor(SSD1306_WIDTH - 1 - (1 * Font_7x10.width), LINE4_Y);
	snprintf(buffer_display, sizeof(buffer_display), (settings.flags & SETTING_FLAG_UART_DSO) ? "U" : " ");       // ON/OFF
	ssd1306_WriteString(buffer_display, &Font_7x10, White);

	// ***************************
	// send the display buffer to the screen

	ssd1306_UpdateScreen();

	draw_screen_count++;
}

// ***********************************************************

int _write(int file, char *ptr, int len)
{
	#if 0
		const HAL_StatusTypeDef res = HAL_UART_Transmit(&huart1, (uint8_t *)ptr, len, 100);
		if (res != HAL_OK)
			len = 0;
	#else
		// start sending using DMA mode
		// wait for a maximum of 100ms for the send to start
		const uint32_t tick = HAL_GetTick();
		while (HAL_BUSY == HAL_UART_Transmit_DMA(&huart1, (uint8_t *)ptr, len) && (HAL_GetTick() - tick) < 100)
			__WFI();    // wait until next interrupt occurs
	#endif

	return len;
}

void flash_led(const uint32_t ms)
{
	__disable_irq();
	while (1)
	{
		HAL_GPIO_WritePin(LED_pin_GPIO_Port, LED_Pin, GPIO_PIN_SET);	// LED on
		DWT_Delay_us(10);
		HAL_GPIO_WritePin(LED_pin_GPIO_Port, LED_Pin, GPIO_PIN_RESET);	// LED off
		DWT_Delay_us(ms * 1000);
	}
}

void Error_Handler(void)
{
	flash_led(1000);
}

#ifdef USE_FULL_ASSERT
	void assert_failed(uint8_t *file, uint32_t line)
	{
	}
#endif

void NMI_Handler(void)
{
	while (1)
	{
	}
}

void HardFault_Handler(void)
{
	flash_led(80);
}

void MemManage_Handler(void)
{
	flash_led(100);
}

void BusFault_Handler(void)
{
	flash_led(100);
}

void UsageFault_Handler(void)
{
	flash_led(100);
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

void SysTick_Handler(void)
{
	HAL_IncTick();

	const uint32_t tick = HAL_GetTick();

	{	// debounce the push buttons

		const int16_t debounce_ms = 30;

		for (unsigned int i = 0; i < BUTTON_NUM; i++)
		{
			t_button *butt = &button[i];
			if (butt->gpio_port == NULL)
				continue;

			// update debounce counter
			int16_t debounce = butt->debounce;
			//debounce = (LL_GPIO_IsInputPinSet(butt->gpio_port, butt->gpio_pin) ==              0) ? debounce + 1 : debounce - 1;
			debounce   = (HAL_GPIO_ReadPin(     butt->gpio_port, butt->gpio_pin) == GPIO_PIN_RESET) ? debounce + 1 : debounce - 1;
			debounce   = (debounce < 0) ? 0 : (debounce > debounce_ms) ? debounce_ms : debounce;
			butt->debounce = debounce;

			const uint32_t pressed_tick = butt->pressed_tick;

			if (pressed_tick == 0 && debounce >= debounce_ms)
			{	// just pressed
				butt->released         = 0;                             // clear released flag
				butt->held_ms          = 0;                             // reset held down time
				butt->processed        = 0;                             // clear flag
				butt->pressed_tick     = tick;                          // remember the tick the button was pressed
			}
			else
			if (pressed_tick > 0 && debounce <= 0)
			{	// just released
				if (butt->processed)
				{
					butt->held_ms      = 0;
					butt->released     = 0;
					butt->pressed_tick = 0;
					butt->processed    = 0;
				}
				else
				{
					butt->held_ms      = tick - pressed_tick;           // save the time the button was held down for
					butt->released     = 1;                             // set released flag
					butt->pressed_tick = 0;                             // reset pressed tick
				}
			}
			else
			if (pressed_tick > 0) // && butt->processed == 0)
			{
				butt->held_ms      = tick - pressed_tick;               // time the button has been held down for
			}
			else
			if (butt->processed)
			{
				butt->held_ms      = 0;
				butt->released     = 0;
				butt->pressed_tick = 0;
				butt->processed    = 0;
			}
		}
	}

	if (save_settings_timer > 0)
		save_settings_timer--;

	#ifdef USE_IWDG
		if (iwdg_tick < (iwdg_tick + 1))
			iwdg_tick++;
	#endif
}

void DMA1_Channel1_IRQHandler(void)
{
	HAL_DMA_IRQHandler(&hdma_adc1);
}

void DMA1_Channel4_IRQHandler(void)
{
	HAL_DMA_IRQHandler(&hdma_usart1_tx);
}

void TIM3_IRQHandler(void)
{
	HAL_TIM_IRQHandler(&htim3);

	// we come here on each and every ADC sample, so it's a good time to set the DAC output value
	//
	// we could also do the ADC sampling here too if we really wanted (manually read the ADC register(s))
	// but that's being done via the DMA, which is just fine

	unsigned int index = sine_table_index;

	DAC_write(sine_table[index]);

	// save updated index
	sine_table_index = (++index >= ARRAY_SIZE(sine_table)) ? 0 : index;
}

void USART1_IRQHandler(void)
{
	HAL_UART_IRQHandler(&huart1);
}

void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *hadc)
{
	if (hadc->Instance == ADC1)
		process_ADC(&adc_dma_buffer[0]);  // lower half of ADC buffer
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
	if (hadc->Instance == ADC1)
		process_ADC(&adc_dma_buffer[1]);  // upper half of ADC buffer
}

// ***********************************************************

void HAL_MspInit(void)
{
	__HAL_RCC_AFIO_CLK_ENABLE();
	__HAL_RCC_PWR_CLK_ENABLE();

	__HAL_AFIO_REMAP_SWJ_NOJTAG();
}

void HAL_ADC_MspInit(ADC_HandleTypeDef *hadc)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	if (hadc->Instance == ADC1)
	{
		__HAL_RCC_ADC1_CLK_ENABLE();

		__HAL_RCC_GPIOA_CLK_ENABLE();

		GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
		GPIO_InitStruct.Pin  = ADC1_Pin;
		HAL_GPIO_Init(ADC1_pin_GPIO_Port, &GPIO_InitStruct);
		GPIO_InitStruct.Pin  = ADC2_Pin;
		HAL_GPIO_Init(ADC2_pin_GPIO_Port, &GPIO_InitStruct);

		hdma_adc1.Instance                 = DMA1_Channel1;
		hdma_adc1.Init.Direction           = DMA_PERIPH_TO_MEMORY;
		hdma_adc1.Init.PeriphInc           = DMA_PINC_DISABLE;
		hdma_adc1.Init.MemInc              = DMA_MINC_ENABLE;
		#ifdef DUAL_ADC_MODE
			hdma_adc1.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
			hdma_adc1.Init.MemDataAlignment    = DMA_MDATAALIGN_WORD;
		#else
			hdma_adc1.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
			hdma_adc1.Init.MemDataAlignment    = DMA_MDATAALIGN_HALFWORD;
		#endif
		hdma_adc1.Init.Mode                = DMA_CIRCULAR;
		hdma_adc1.Init.Priority            = DMA_PRIORITY_HIGH;
		if (HAL_DMA_Init(&hdma_adc1) != HAL_OK)
			Error_Handler();

		__HAL_LINKDMA(hadc, DMA_Handle, hdma_adc1);
	}
	#ifdef DUAL_ADC_MODE
		else
		if (hadc->Instance == ADC2)
			__HAL_RCC_ADC2_CLK_ENABLE();
	#endif
}

void HAL_ADC_MspDeInit(ADC_HandleTypeDef *hadc)
{
	if (hadc->Instance == ADC1)
	{
		__HAL_RCC_ADC1_CLK_DISABLE();
		HAL_GPIO_DeInit(ADC1_pin_GPIO_Port, ADC1_Pin);
		HAL_GPIO_DeInit(ADC2_pin_GPIO_Port, ADC2_Pin);
		HAL_DMA_DeInit(hadc->DMA_Handle);
		HAL_NVIC_DisableIRQ(ADC1_2_IRQn);
	}
	#ifdef DUAL_ADC_MODE
		else
		if (hadc->Instance == ADC2)
		{
			__HAL_RCC_ADC2_CLK_DISABLE();
			HAL_NVIC_DisableIRQ(ADC1_2_IRQn);
		}
	#endif
}

void HAL_TIM_Base_MspInit(TIM_HandleTypeDef *htim_base)
{
	if (htim_base->Instance == TIM3)
	{
		__HAL_RCC_TIM3_CLK_ENABLE();

		HAL_NVIC_SetPriority(TIM3_IRQn, 0, 0);    // Preempt = 0, Sub = 0
		HAL_NVIC_EnableIRQ(TIM3_IRQn);
	}
}

void HAL_TIM_Base_MspDeInit(TIM_HandleTypeDef *htim_base)
{
	if (htim_base->Instance == TIM3)
		__HAL_RCC_TIM3_CLK_DISABLE();
}

void HAL_UART_MspInit(UART_HandleTypeDef *huart)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	if (huart->Instance == USART1)
	{
		__HAL_RCC_USART1_CLK_ENABLE();

		__HAL_RCC_GPIOA_CLK_ENABLE();

		GPIO_InitStruct.Mode  = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
		GPIO_InitStruct.Pin   = UART1_TXD_Pin;
		HAL_GPIO_Init(UART1_TXD_GPIO_Port, &GPIO_InitStruct);

		GPIO_InitStruct.Mode  = GPIO_MODE_INPUT;
		GPIO_InitStruct.Pull  = GPIO_PULLUP;
		GPIO_InitStruct.Pin   = UART1_RXD_Pin;
		HAL_GPIO_Init(UART1_RXD_GPIO_Port, &GPIO_InitStruct);

		hdma_usart1_tx.Instance                 = DMA1_Channel4;
		hdma_usart1_tx.Init.Direction           = DMA_MEMORY_TO_PERIPH;
		hdma_usart1_tx.Init.PeriphInc           = DMA_PINC_DISABLE;
		hdma_usart1_tx.Init.MemInc              = DMA_MINC_ENABLE;
		hdma_usart1_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
		hdma_usart1_tx.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
		hdma_usart1_tx.Init.Mode                = DMA_NORMAL;
		hdma_usart1_tx.Init.Priority            = DMA_PRIORITY_LOW;
		if (HAL_DMA_Init(&hdma_usart1_tx) != HAL_OK)
		{
			Error_Handler();
		}

		__HAL_LINKDMA(huart, hdmatx, hdma_usart1_tx);

		HAL_NVIC_SetPriority(USART1_IRQn, 10, 0);
		HAL_NVIC_EnableIRQ(USART1_IRQn);
	}
}

void HAL_UART_MspDeInit(UART_HandleTypeDef *huart)
{
	if (huart->Instance == USART1)
	{
		__HAL_RCC_USART1_CLK_DISABLE();
		HAL_GPIO_DeInit(UART1_TXD_GPIO_Port, UART1_TXD_Pin);
		HAL_GPIO_DeInit(UART1_RXD_GPIO_Port, UART1_RXD_Pin);
		HAL_DMA_DeInit(huart->hdmatx);
		HAL_NVIC_DisableIRQ(USART1_IRQn);
	}
}

void SystemClock_Config(void)
{
	RCC_OscInitTypeDef       RCC_OscInitStruct = {0};
	RCC_ClkInitTypeDef       RCC_ClkInitStruct = {0};
	RCC_PeriphCLKInitTypeDef PeriphClkInit     = {0};

	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState       = RCC_HSE_ON;
	RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
	RCC_OscInitStruct.HSIState       = RCC_HSI_ON;
	RCC_OscInitStruct.PLL.PLLState   = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource  = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLMUL     = RCC_PLL_MUL9;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		Error_Handler();
	}

	RCC_ClkInitStruct.ClockType      = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
	{
		Error_Handler();
	}

	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
	PeriphClkInit.AdcClockSelection    = RCC_ADCPCLK2_DIV6;
	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
	{
		Error_Handler();
	}
}

void UART1_init(void)
{
	huart1.Instance          = USART1;
	huart1.Init.BaudRate     = UART_BAUDRATE;
	huart1.Init.WordLength   = UART_WORDLENGTH_8B;
	huart1.Init.StopBits     = UART_STOPBITS_1;
	huart1.Init.Parity       = UART_PARITY_NONE;
	huart1.Init.Mode         = UART_MODE_TX_RX;
	huart1.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
	huart1.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart1) != HAL_OK)
	{
		Error_Handler();
	}
}

void ADC_init(void)
{
	ADC_ChannelConfTypeDef sConfig = {0};

	#ifdef DUAL_ADC_MODE
		// simultaneous dual ADC mode

		// ************************************************
		// setup the master ADC

		hadc1.Instance                   = ADC1;
		hadc1.Init.ScanConvMode          = ADC_SCAN_DISABLE;
		hadc1.Init.ContinuousConvMode    = DISABLE;
		hadc1.Init.DiscontinuousConvMode = DISABLE;
		hadc1.Init.ExternalTrigConv      = ADC_EXTERNALTRIGCONV_T3_TRGO;
		hadc1.Init.DataAlign             = ADC_DATAALIGN_RIGHT;
		hadc1.Init.NbrOfConversion       = 1;
		if (HAL_ADC_Init(&hadc1) != HAL_OK)
			Error_Handler();

		//  Configure the ADC multi-mode
		ADC_MultiModeTypeDef multimode = {0};
		multimode.Mode                 = ADC_DUALMODE_REGSIMULT;
		if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
			Error_Handler();

//		sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLES_5;     //   1.5 + 12.5 =  14 cycles, ADC clk = 12MHz, 1.2us sample time, max 857kHz sample rate
//		sConfig.SamplingTime = ADC_SAMPLETIME_7CYCLES_5;     //   7.5 + 12.5 =  20 cycles, ADC clk = 12MHz, 1.7us sample time, max 600kHz sample rate
//		sConfig.SamplingTime = ADC_SAMPLETIME_13CYCLES_5;    //  13.5 + 12.5 =  26 cycles, ADC clk = 12MHz, 2.2us sample time, max 461kHz sample rate
//		sConfig.SamplingTime = ADC_SAMPLETIME_28CYCLES_5;    //  28.5 + 12.5 =  41 cycles, ADC clk = 12MHz, 3.4us sample time, max 292kHz sample rate
//		sConfig.SamplingTime = ADC_SAMPLETIME_41CYCLES_5;    //  41.5 + 12.5 =  54 cycles, ADC clk = 12MHz, 4.5us sample time, max 222kHz sample rate
//		sConfig.SamplingTime = ADC_SAMPLETIME_55CYCLES_5;    //  55.5 + 12.5 =  68 cycles, ADC clk = 12MHz, 5.7us sample time, max 176kHz sample rate
		sConfig.SamplingTime = ADC_SAMPLETIME_71CYCLES_5;    //  71.5 + 12.5 =  84 cycles, ADC clk = 12MHz, 7.0us sample time, max 142kHz sample rate
//		sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;   // 239.5 + 12.5 = 252 cycles, ADC clk = 12MHz, 21us sample time, max 47.6kHz sample rate

		// Configure Regular Channel
		sConfig.Channel = ADC_CHANNEL_0;                // PA0 Pin
		sConfig.Rank    = ADC_REGULAR_RANK_1;
		if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
			Error_Handler();

		// ************************************************
		// setup the slave ADC

		hadc2.Instance                = ADC2;
		hadc2.Init                    = hadc1.Init;
		hadc2.Init.ContinuousConvMode = ENABLE;              // **Very Important (Not Mentioned in any Document)
		hadc2.Init.ExternalTrigConv   = ADC_SOFTWARE_START;
		if (HAL_ADC_Init(&hadc2) != HAL_OK)
			Error_Handler();

		// Configure Regular Channel
		sConfig.Channel = ADC_CHANNEL_1;              // PA1 Pin
		sConfig.Rank    = ADC_REGULAR_RANK_1;
		if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
			Error_Handler();

		// ************************************************
		// run the ADC calibrations

		if (HAL_ADCEx_Calibration_Start(&hadc1) != HAL_OK)
			Error_Handler();

		if (HAL_ADCEx_Calibration_Start(&hadc2) != HAL_OK)
			Error_Handler();

	#else
		// single ADC dual channel mode

		hadc1.Instance                   = ADC1;
		hadc1.Init.ScanConvMode          = ADC_SCAN_ENABLE;
		hadc1.Init.ContinuousConvMode    = DISABLE;
		hadc1.Init.DiscontinuousConvMode = DISABLE;
		hadc1.Init.ExternalTrigConv      = ADC_EXTERNALTRIGCONV_T3_TRGO;
		hadc1.Init.DataAlign             = ADC_DATAALIGN_RIGHT;
		hadc1.Init.NbrOfConversion       = 2;
		if (HAL_ADC_Init(&hadc1) != HAL_OK)
			Error_Handler();

//		sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLES_5;     //   1.5 + 12.5 =  14 cycles, ADC clk = 12MHz, 1.2us sample time, max 857kHz sample rate
//		sConfig.SamplingTime = ADC_SAMPLETIME_7CYCLES_5;     //   7.5 + 12.5 =  20 cycles, ADC clk = 12MHz, 1.7us sample time, max 600kHz sample rate
//		sConfig.SamplingTime = ADC_SAMPLETIME_13CYCLES_5;    //  13.5 + 12.5 =  26 cycles, ADC clk = 12MHz, 2.2us sample time, max 461kHz sample rate
//		sConfig.SamplingTime = ADC_SAMPLETIME_28CYCLES_5;    //  28.5 + 12.5 =  41 cycles, ADC clk = 12MHz, 3.4us sample time, max 292kHz sample rate
//		sConfig.SamplingTime = ADC_SAMPLETIME_41CYCLES_5;    //  41.5 + 12.5 =  54 cycles, ADC clk = 12MHz, 4.5us sample time, max 222kHz sample rate
//		sConfig.SamplingTime = ADC_SAMPLETIME_55CYCLES_5;    //  55.5 + 12.5 =  68 cycles, ADC clk = 12MHz, 5.7us sample time, max 176kHz sample rate
		sConfig.SamplingTime = ADC_SAMPLETIME_71CYCLES_5;    //  71.5 + 12.5 =  84 cycles, ADC clk = 12MHz, 7.0us sample time, max 142kHz sample rate
//		sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;   // 239.5 + 12.5 = 252 cycles, ADC clk = 12MHz, 21us sample time, max 47.6kHz sample rate

		sConfig.Channel = ADC_CHANNEL_0;                // PA0 Pin
		sConfig.Rank    = ADC_REGULAR_RANK_1;
		if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
			Error_Handler();

		sConfig.Channel = ADC_CHANNEL_1;                // PA1 Pin
		sConfig.Rank    = ADC_REGULAR_RANK_2;
		if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
			Error_Handler();

		// ************************************************
		// run the ADC calibration

		if (HAL_ADCEx_Calibration_Start(&hadc1) != HAL_OK)
			Error_Handler();

	#endif
}

void DMA_init(void)
{
	__HAL_RCC_DMA1_CLK_ENABLE();

	// ADC DMA
	HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 1, 0);
	HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

	// UART TX DMA
	HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 10, 0);
	HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);
}

void TIMER3_init(void)
{
	TIM_ClockConfigTypeDef  sClockSourceConfig = {0};
	TIM_MasterConfigTypeDef sMasterConfig      = {0};

//	RCC_ClkInitTypeDef rcc_clk;
//	uint32_t latency;
//	HAL_RCC_GetClockConfig(&rcc_clk, &latency);

	const uint32_t timer_rate_Hz = (ADC_DATA_LENGTH / 2) * 1000;      // 1kHz

	htim3.Instance               = TIM3;
	htim3.Init.CounterMode       = TIM_COUNTERMODE_UP;
	htim3.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
	htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
	htim3.Init.Prescaler         = 0;
	htim3.Init.Period            = (((HAL_RCC_GetHCLKFreq() / (htim3.Init.Prescaler + 1)) + (timer_rate_Hz / 2)) / timer_rate_Hz) - 1;
	if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
		Error_Handler();

	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
		Error_Handler();

	sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
	sMasterConfig.MasterSlaveMode     = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
		Error_Handler();
}

void GPIO_init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	// LED on
	HAL_GPIO_WritePin(LED_pin_GPIO_Port, LED_Pin, HIGH);

	// voltage measurement mode
	HAL_GPIO_WritePin(VI_pin_GPIO_Port, VI_Pin, LOW);
	HAL_GPIO_WritePin(GS_pin_GPIO_Port, GS_Pin, HIGH);

	HAL_GPIO_WritePin(TP21_pin_GPIO_Port, TP21_Pin, LOW);
	HAL_GPIO_WritePin(TP22_pin_GPIO_Port, TP22_Pin, LOW);

	DAC_write(0);

	// output pins
	GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull  = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.Pin   = LED_Pin;
	HAL_GPIO_Init(LED_pin_GPIO_Port, &GPIO_InitStruct);
	GPIO_InitStruct.Pin   = GS_Pin;
	HAL_GPIO_Init(GS_pin_GPIO_Port, &GPIO_InitStruct);
	GPIO_InitStruct.Pin   = VI_Pin;
	HAL_GPIO_Init(VI_pin_GPIO_Port, &GPIO_InitStruct);
	GPIO_InitStruct.Pin   = TP21_Pin;
	HAL_GPIO_Init(TP21_pin_GPIO_Port, &GPIO_InitStruct);
	GPIO_InitStruct.Pin   = TP22_Pin;
	HAL_GPIO_Init(TP22_pin_GPIO_Port, &GPIO_InitStruct);

	// input pins
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Pin  = BUTT_HOLD_Pin;
	HAL_GPIO_Init(BUTT_HOLD_GPIO_Port, &GPIO_InitStruct);
	GPIO_InitStruct.Pin  = BUTT_SP_Pin;
	HAL_GPIO_Init(BUTT_SP_GPIO_Port, &GPIO_InitStruct);
	GPIO_InitStruct.Pin  = BUTT_RCL_Pin;
	HAL_GPIO_Init(BUTT_RCL_GPIO_Port, &GPIO_InitStruct);

	// DAC pins
	GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull  = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	GPIO_InitStruct.Pin   = DA0_Pin;
	HAL_GPIO_Init(DA0_pin_GPIO_Port, &GPIO_InitStruct);
	GPIO_InitStruct.Pin   = DA1_Pin;
	HAL_GPIO_Init(DA1_pin_GPIO_Port, &GPIO_InitStruct);
	GPIO_InitStruct.Pin   = DA2_Pin;
	HAL_GPIO_Init(DA2_pin_GPIO_Port, &GPIO_InitStruct);
	GPIO_InitStruct.Pin   = DA3_Pin;
	HAL_GPIO_Init(DA3_pin_GPIO_Port, &GPIO_InitStruct);
	GPIO_InitStruct.Pin   = DA4_Pin;
	HAL_GPIO_Init(DA4_pin_GPIO_Port, &GPIO_InitStruct);
	GPIO_InitStruct.Pin   = DA5_Pin;
	HAL_GPIO_Init(DA5_pin_GPIO_Port, &GPIO_InitStruct);
	GPIO_InitStruct.Pin   = DA6_Pin;
	HAL_GPIO_Init(DA6_pin_GPIO_Port, &GPIO_InitStruct);
	GPIO_InitStruct.Pin   = DA7_Pin;
	HAL_GPIO_Init(DA7_pin_GPIO_Port, &GPIO_InitStruct);
}

// ***********************************************************

void process_buttons(void)
{
	// both HOLD and S/P buttons held down
	if (button[BUTTON_HOLD].pressed_tick > 0 && button[BUTTON_SP].pressed_tick > 0 && button[BUTTON_RCL].pressed_tick == 0)
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

	if (button[BUTTON_HOLD].pressed_tick > 0 && button[BUTTON_SP].pressed_tick == 0 && button[BUTTON_RCL].pressed_tick == 0)
	{
		if (button[BUTTON_HOLD].held_ms >= 500 && button[BUTTON_HOLD].processed == 0)
		{	// HOLD held down
			button[BUTTON_HOLD].processed = 1;

			display_hold = 0;
			memset((void *)&calibrate, 0, sizeof(calibrate));
			calibrate.Hz = 100;
			op_mode = OP_MODE_OPEN_PROBE_CALIBRATION;
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
			//settings.flags ^= SETTING_FLAG_UART_DSO;

			draw_screen();
		}
		return;
	}

	// *************
	// S/P button

	if (button[BUTTON_HOLD].pressed_tick == 0 && button[BUTTON_SP].pressed_tick > 0 && button[BUTTON_RCL].pressed_tick == 0)
	{
		if (button[BUTTON_SP].held_ms >= 500 && button[BUTTON_SP].processed == 0)
		{	// S/P held down
			button[BUTTON_SP].processed = 1;

			display_hold = 0;
			memset((void *)&calibrate, 0, sizeof(calibrate));
			calibrate.Hz = 100;
			op_mode = OP_MODE_SHORTED_PROBE_CALIBRATION;
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

	if (button[BUTTON_HOLD].pressed_tick == 0 && button[BUTTON_SP].pressed_tick == 0 && button[BUTTON_RCL].pressed_tick > 0)
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

void process_uart_send(void)
{
	if (settings.flags & SETTING_FLAG_UART_DSO)
	{
		const HAL_UART_StateTypeDef state = HAL_UART_GetState(&huart1);
		if (state == HAL_UART_STATE_READY)
		{	// the UART is available to use
			//
			// send the sampled data down the serial link

			#ifdef MATLAB_SERIAL
				// send as ASCII

				const unsigned int cols = 8;
				for (unsigned int i = 0; i < ADC_DATA_LENGTH; i++)
				{
					printf("%3u,", 1 + i);
					for (unsigned int col = 0; col < (cols - 1); col++)
						printf("%0.1f,", adc_data[col][i]);
					printf("%0.1f\r\n", adc_data[col - 1][i]);
				}
			#else
				// send as binary packet

				// STM32's are little endian (data is LS-Byte 1st)
				// the receiving end needs to take that into account when processing the rx'ed data
				// your receiving app can use htons(), htonl(), ntohs(), ntohl() to swap endianness (if need be)

				// create TX packet
				tx_packet.marker = PACKET_MARKER;                                         // packet start marker
				memcpy(tx_packet.data, &adc_data, sizeof(adc_data));                      // packet data

				#ifdef UART_BIG_ENDIAN
					// make the packet values BIG endian
					// though the receivng end (your PC etc) is the one that needs to be dealing with this, not us

					tx_packet.marker = __builtin_bswap32(tx_packet.marker);

					uint32_t *pd = (uint32_t *)tx_packet.data;
					for (unsigned int i = 0; i < (sizeof(adc_data) / sizeof(uint32_t)); i++, pd++)
						*pd = __builtin_bswap32(*pd);
				#endif

				//HAL_GPIO_WritePin(TP21_pin_GPIO_Port, TP21_Pin, LOW);

				#if 0
					tx_packet.crc = 0;    // TEST
				#else
					tx_packet.crc = CRC16_block(0, tx_packet.data, sizeof(tx_packet.data));   // packet CRC - compute the CRC of the data
				#endif

				#ifdef UART_BIG_ENDIAN
					// make the packet CRC little endian
					tx_packet.crc = __builtin_bswap16(tx_packet.crc);
				#endif

				#if 0
					// start sending the packet (wait here for upto 200ms until it does start)
					const uint32_t tick = HAL_GetTick();
					while (HAL_BUSY == HAL_UART_Transmit_DMA(&huart1, (uint8_t *)&tx_packet, sizeof(tx_packet)) && (HAL_GetTick() - tick) < 200)
						__WFI();    // wait until next interrupt occurs
				#else
					// don't hang around here waiting, if the UART/DMA is still busy then forget sending it on this run
					HAL_UART_Transmit_DMA(&huart1, (uint8_t *)&tx_packet, sizeof(tx_packet));
				#endif
			#endif
		}
	}
}

void process_op_mode(void)
{
	switch (op_mode)
	{
		default:
		case OP_MODE_MEASURING:                // normal measurement mode
			break;

		case OP_MODE_OPEN_PROBE_CALIBRATION:   // doing an OPEN probe calibration
		{
			for (unsigned int i = 0; i < ARRAY_SIZE(calibrate.mag_sum); i++)
				calibrate.mag_sum[i] += mag_rms[i];

			for (unsigned int i = 0; i < ARRAY_SIZE(calibrate.phase_sum); i++)
			{
				const float phase_rad = phase_deg[i] * DEG_TO_RAD;
				calibrate.phase_sum[i].re += cosf(phase_rad);
				calibrate.phase_sum[i].im += sinf(phase_rad);
			}

			save_settings_timer = SAVE_SETTINGS_MS;   // delay saving the settings

			if (++calibrate.count >= CALIBRATE_COUNT)
			{
				const unsigned int index = (calibrate.Hz == 100) ? 0 : 1;   // 100Hz/1kHz

				for (unsigned int i = 0; i < ARRAY_SIZE(calibrate.mag_sum); i++)
					settings.open_probe_calibration[index].mag_rms[i] = calibrate.mag_sum[i] / calibrate.count;

				for (unsigned int i = 0; i < ARRAY_SIZE(calibrate.phase_sum); i++)
					settings.open_probe_calibration[index].phase_deg[i] = (calibrate.phase_sum[i].re != 0.0f) ? atan2f(calibrate.phase_sum[i].im, calibrate.phase_sum[i].re) * RAD_TO_DEG : NAN;

				settings.open_probe_calibration[index].done = 1;

				if (index == 0)
				{	// do the same again but at the next measurement frequency

					memset((void *)&calibrate, 0, sizeof(calibrate));
					calibrate.Hz = 1000;
				}
				else
				{	// done

					// restore original measurement frequency
					set_measurement_frequency(settings.measurement_Hz);

					op_mode = OP_MODE_MEASURING;
				}

				draw_screen();
			}

			break;
		}

		case OP_MODE_SHORTED_PROBE_CALIBRATION:   // doing a SHORTED probe calibration
		{
			for (unsigned int i = 0; i < ARRAY_SIZE(calibrate.mag_sum); i++)
				calibrate.mag_sum[i] += mag_rms[i];

			for (unsigned int i = 0; i < ARRAY_SIZE(calibrate.phase_sum); i++)
			{
				const float phase_rad = phase_deg[i] * DEG_TO_RAD;
				calibrate.phase_sum[i].re += cosf(phase_rad);
				calibrate.phase_sum[i].im += sinf(phase_rad);
			}

			save_settings_timer = SAVE_SETTINGS_MS;   // delay saving the settings

			if (++calibrate.count >= CALIBRATE_COUNT)
			{
				const unsigned int index = (calibrate.Hz == 100) ? 0 : 1;   // 100Hz/1kHz

				for (unsigned int i = 0; i < ARRAY_SIZE(calibrate.mag_sum); i++)
					settings.shorted_probe_calibration[index].mag_rms[i] = calibrate.mag_sum[i] / calibrate.count;

				for (unsigned int i = 0; i < ARRAY_SIZE(calibrate.phase_sum); i++)
					settings.shorted_probe_calibration[index].phase_deg[i] = (calibrate.phase_sum[i].re != 0.0f) ? atan2f(calibrate.phase_sum[i].im, calibrate.phase_sum[i].re) * RAD_TO_DEG : NAN;

				settings.shorted_probe_calibration[index].done = 1;

				if (index == 0)
				{	// do the same again but at the next measurement frequency

					memset((void *)&calibrate, 0, sizeof(calibrate));
					calibrate.Hz = 1000;
				}
				else
				{	// done

					// restore original measurement frequency
					set_measurement_frequency(settings.measurement_Hz);

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
	// free PB3 and PB4 for general use
//	AFIO->MAPR &= ~(AFIO_MAPR_SWJ_CFG);   // Clear the SWJ_CFG bits
//	AFIO->MAPR |=   AFIO_MAPR_SWJ_CFG_1;  // Set SWJ_CFG to disable JTAG but keep SWD

	#pragma GCC diagnostic push
	#pragma GCC diagnostic ignored "-Wunused-variable"
	// -Wunused -Wunused-function -Wunused-label -Wunused-parameter -Wunused-value -Wunused-variable -Wunused-but-set-parameter -Wunused-but-set-variable
	reset_cause.por    = (__HAL_RCC_GET_FLAG(RCC_FLAG_PORRST ) != RESET) ? 1 : 0;
	reset_cause.pin    = (__HAL_RCC_GET_FLAG(RCC_FLAG_PINRST ) != RESET) ? 1 : 0;
	reset_cause.sft    = (__HAL_RCC_GET_FLAG(RCC_FLAG_SFTRST ) != RESET) ? 1 : 0;
	reset_cause.iwdg   = (__HAL_RCC_GET_FLAG(RCC_FLAG_IWDGRST) != RESET) ? 1 : 0;
	reset_cause.wwdg   = (__HAL_RCC_GET_FLAG(RCC_FLAG_WWDGRST) != RESET) ? 1 : 0;
	reset_cause.lpwr   = (__HAL_RCC_GET_FLAG(RCC_FLAG_LPWRRST) != RESET) ? 1 : 0;
	reset_cause.lsirdy = (__HAL_RCC_GET_FLAG(RCC_FLAG_LSIRDY ) != RESET) ? 1 : 0;
	__HAL_RCC_CLEAR_RESET_FLAGS();		// reset flags ready for next reboot
	#pragma GCC diagnostic pop

	// disable printf(), fread(), fwrite(), sscanf() etc buffering
	setbuf( stdin,  NULL);
	setbuf( stdout, NULL);
	setbuf( stderr, NULL);
//	setvbuf(stdout, NULL, _IONBF, 0);
//	setvbuf(stderr, NULL, _IONBF, 0);

	button[0].gpio_port = BUTT_HOLD_GPIO_Port;
	button[0].gpio_pin  = BUTT_HOLD_Pin;

	button[1].gpio_port = BUTT_SP_GPIO_Port;
	button[1].gpio_pin  = BUTT_SP_Pin;

	button[2].gpio_port = BUTT_RCL_GPIO_Port;
	button[2].gpio_pin  = BUTT_RCL_Pin;

	// defaults
	settings.series_ohms    = SERIES_RESISTOR_OHMS;    // this can be calibrated using a DUT with a known resistance value
	settings.measurement_Hz = 1000;
//	settings.lcr_mode       = LCR_MODE_INDUCTANCE;
//	settings.lcr_mode       = LCR_MODE_CAPACITANCE;
	settings.lcr_mode       = LCR_MODE_RESISTANCE;
//	settings.flags          = 0;
	settings.flags         |= SETTING_FLAG_UART_DSO;   // send ADC data via the serial port

	DWT_Delay_Init();
	HAL_Init();
	SystemClock_Config();

	#ifdef USE_IWDG
		MX_IWDG_Init();
	#endif

	GPIO_init();
	DMA_init();     // must do this before uart/dma is setup, otherwise the uart tx dma doesn't work
	UART1_init();
	TIMER3_init();  // Timer-3 is used for the ADC and DAC output
	ADC_init();

	{	// setup the goertzel filter
		const float normalized_freq = 2.0f / ADC_DATA_LENGTH;  // 2 cycles spanning the sample buffer
		goertzel_init(&goertzel, normalized_freq);
	}

	// fetch saved settings from flash
	read_settings();

	inv_series_ohms = 1.0f / settings.series_ohms;

	system_data.vi_measure_mode = vi_measure_mode_table[vi_measure_index];
	set_measure_mode_pins(system_data.vi_measure_mode);

	set_measurement_frequency(settings.measurement_Hz);

	screen_init();
	bootup_screen();

	printf("\r\nrebooted m181 LCR Meter v%.2f\r\n", FW_VERSION);

	{	// wait until all buttons have been released for at least 500ms
		uint32_t butt_tick = HAL_GetTick();

		while (1)
		{
			__WFI();

			const uint32_t tick = HAL_GetTick();

			for (unsigned int i = 0; i < ARRAY_SIZE(button); i++)
				if (button[i].debounce > 0)
					butt_tick = tick;

			if ((tick - butt_tick) >= 500)
				break;

			#ifdef USE_IWDG
				// feed the dog
				service_IWDG(0);
			#endif
		}

		// clear any butt press info
		for (unsigned int i = 0; i < ARRAY_SIZE(button); i++)
		{
			button[i].debounce     = 0;
			button[i].pressed_tick = 0;
			button[i].released     = 0;
			button[i].held_ms      = 0;
		}
	}

	// start sampling
	start_ADC();

	// give the user more time to read the bootup screen
	HAL_Delay(1000);

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

		process_buttons();

//		const unsigned int prev_vi_measure_mode = system_data.vi_measure_mode;
		system_data.vi_measure_mode = vi_measure_mode_table[vi_measure_index];

		if (vi_measure_index >= VI_MODE_DONE)
		{	// completed another full measurement cycle

			process_data();
			process_op_mode();
			draw_screen();
			process_uart_send();

			frames++;

			vi_measure_index = 0;         // start next data capture
		}
		//else
		//if (system_data.vi_measure_mode != prev_vi_measure_mode && draw_screen_count > 0)
		//	draw_measurement_mode();

		// save settings to flash if it's time too
		if (save_settings_timer == 0)
		{
			if (write_settings() < 0)     // save settings to flash
				write_settings();         // failed, have a 2nd go
			save_settings_timer = -1;     // don't try saving again (until the next time)
		}

		#ifdef USE_IWDG
			service_IWDG(0);
		#endif
	}
}

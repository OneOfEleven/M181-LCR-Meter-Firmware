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

typedef struct {
	GPIO_TypeDef     *gpio_port;
	uint16_t          gpio_pin;
	int16_t           debounce;        // counter for debounce
	volatile uint32_t pressed_tick;    // tick value when button was pressed
	volatile uint8_t  released;        // true when button is released
	volatile uint32_t held_ms;         // number of ms the button is/was held for
} t_button;

typedef struct {
	int16_t adc;
	int16_t afc;
} t_adc_dma_data_16;

typedef struct {
	int32_t adc;
	int32_t afc;
} t_adc_dma_data_32;

typedef struct {
	float re;
	float im;
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
		float   data[DMA_ADC_DATA_LENGTH * 8];
		uint8_t data_b[sizeof(float) * DMA_ADC_DATA_LENGTH * 8];
	};
} t_packet;
#pragma pack(pop)

static const uint16_t omega_7x10[10] = {
	0x001C, 0x0022, 0x0041, 0x0041, 0x0041, 0x0041, 0x0022, 0x001C, 0x0014, 0x0022
};

#if defined(USE_IWDG) && defined(HAL_IWDG_MODULE_ENABLED)
	IWDG_HandleTypeDef hiwdg      = {0};
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
char                  buffer_display[20] = {0};

t_button              button[3] = {0};

const uint16_t        DAC_resolution                      = 256;  // 8-bit
float                 sine_wave_amplitude                 = 1.0;  // 0.0 = 0%, 1.0 = 100%, -1.0 = 100% phase inverted
volatile unsigned int sine_table_index                    = 0;    //
uint8_t               sine_table[DMA_ADC_DATA_LENGTH / 2] = {0};  // matched to the ADC sampling

unsigned int          op_mode = OP_MODE_MEASURING;

// for the calibration modes
struct {
	int               count;
	float             mag_sum[8];
	t_comp            phase_sum[8];
} calibrate = {0};

unsigned int          volt_gain_sel = 0;
unsigned int          amp_gain_sel  = 0;

int8_t                ref0_data[DMA_ADC_DATA_LENGTH]  = {0};
int8_t                ref90_data[DMA_ADC_DATA_LENGTH] = {0};

int16_t               phase_offset_array_index = 0;

// raw ADC sample buffer
t_adc_dma_data_16     raw_adc_dma_data[2][DMA_ADC_DATA_LENGTH];      // *2 for double buffering (ADC/DMA is continuously running)

// ADC sample block averaging buffer
// we take several sample blocks and average them together to reduce noise/increase dynamic range
unsigned int          adc_average_count                 = DEFAULT_ADC_AVERAGE_COUNT;
t_adc_dma_data_32     adc_data_avg[DMA_ADC_DATA_LENGTH] = {0};
unsigned int          adc_data_avg_count                = 0;

// ADC rolling average
t_comp                adc_avg[8]    = {0};
uint32_t              adc_avg_count = 0;

#pragma pack(push, 1)
float                 adc_data[8][DMA_ADC_DATA_LENGTH] = {0};
#pragma pack(pop)

float                 mag_rms[8]   = {0};
float                 phase_deg[8] = {0};

// custom HW VI mode sequence order to minimize mode switching time
// because of a HW design floor (takes time for HW to settle after changing the HW GS/VI mode pins)
const unsigned int    vi_measure_mode_table[] = {0, 1, 3, 2};
volatile unsigned int vi_measure_index = 0;
unsigned int          prev_vi_mode = -1;

// various system settings and results
volatile int          save_settings_timer = -1;
t_settings            settings            = {0};
t_system_data         system_data         = {0};

// temp buffer - used for the Goerttzel output samples
t_comp                tmp_buf[DMA_ADC_DATA_LENGTH];

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

void set_DAC_amplitude(const float amplitude)
{	// fill the look-up buffer with one complete sine cycle
	//
	// 'amplitude' .. -1.0 to +1.0 (negative inverts the phase)

	sine_wave_amplitude = (amplitude < -1.0f) ? -1.0f : (amplitude > 1.0f) ? 1.0f : amplitude;

	const float scale      = (DAC_resolution - 1) * sine_wave_amplitude * 0.5f;
	const float phase_step = (float)(2.0 * M_PI) / ARRAY_SIZE(sine_table);

	for (unsigned int i = 0; i < ARRAY_SIZE(sine_table); i++)
		sine_table[i] = (uint8_t)floorf(((1.0f + sinf(phase_step * i)) * scale) + 0.5f); // raised sine
}

void start_ADC(void)
{
	// non-stop ADC double buffered sampling
	#ifdef DUAL_ADC_MODE
		HAL_ADC_Start(&hadc2);
		HAL_ADCEx_MultiModeStart_DMA(&hadc1, (uint32_t *)raw_adc_dma_data, DMA_ADC_DATA_LENGTH * 2);
	#else
		HAL_ADC_Start_DMA(&hadc1, (uint32_t *)raw_adc_dma_data, DMA_ADC_DATA_LENGTH * 2);
	#endif

	sine_table_index = ARRAY_SIZE(sine_table) * 0.285; // align the sinewave (@1kHz)

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
// emulated eepom in flash
//
// includes wear leveling by saving each successive settings block to a different flash address, the previous saves are left alone.
// once the allocated eeprom flash pages become full, they are all erased in one go ready for more saves to occur.

const __IO uint32_t * find_last_good_settings(void)
{	// find the last valid flash saved settings we did

	const __IO uint32_t *flash_addr_end  = (uint32_t *)EEPROM_END_ADDRESS;
	const __IO uint32_t *flash_addr      = (uint32_t *)EEPROM_START_ADDRESS;  // location of where the settings block are stored in flash
	const __IO uint32_t *flash_addr_good = NULL;                              // flash address of last good block of settings found

	while (1)
	{
		t_settings settings;

		// read a settings block from flash
		const __IO uint32_t *addr = flash_addr;
		uint32_t *p = (uint32_t *)&settings;
		for (unsigned int i = 0; i < sizeof(t_settings); i += sizeof(uint32_t))
			 *p++ = *addr++;

		// validate those settings
		// if good, then remember it's flash address
		if (settings.marker == SETTINGS_MARKER)
		{
			const uint16_t crc1 = settings.crc;
			settings.crc = 0;
			const uint16_t crc2 = CRC16_block(0, &settings, sizeof(t_settings));
			if (crc1 == crc2)
				flash_addr_good = flash_addr;       // marker and CRC appear to be OK (valid settings), remember the blocks flash address
		}

		// point to the next block
		flash_addr = addr;
		if ((flash_addr + (sizeof(t_settings) / sizeof(flash_addr[0]))) > flash_addr_end)
			break;     // no more slots to check, we've reached the end of the allocated eeprom flash pages
	}

	// return the flash address of the last saved block of error-free settings
	return flash_addr_good;
}

int read_settings(void)
{	// read settings from flash

	// fetch the flash address of last saved settings
	const __IO uint32_t *flash_addr = find_last_good_settings();
	if (flash_addr == NULL)
		return -1;           // none found - nothing to load

	// copy them into the system settings
	uint32_t *p = (uint32_t *)&settings;
	for (unsigned int i = 0; i < sizeof(t_settings); i += sizeof(uint32_t))
		 *p++ = *flash_addr++;

	return 0;
}

int write_settings(void)
{	// write settings to flash

	const __IO uint32_t *flash_addr_end = (uint32_t *)EEPROM_END_ADDRESS;

	// use this if flash page erase is required
	FLASH_EraseInitTypeDef erase_init = {0};
	erase_init.TypeErase   = FLASH_TYPEERASE_PAGES;
	erase_init.PageAddress = EEPROM_START_ADDRESS;
	erase_init.NbPages     = (EEPROM_END_ADDRESS - EEPROM_START_ADDRESS) / PAGE_SIZE;
	uint32_t page_error    = 0;

	// add marker and CRC to the current system settings (so we can scan and find them again in flash)
	settings.marker = SETTINGS_MARKER;
	settings.crc = 0;
	settings.crc = CRC16_block(0, &settings, sizeof(t_settings));

	// find where we last saved the previous settings
	const __IO uint32_t *flash_addr = find_last_good_settings();
	if (flash_addr != NULL)
	{	// found them

		// point to the next flash slot - to save into
		flash_addr += sizeof(t_settings) / sizeof(flash_addr[0]);

		if ((flash_addr + (sizeof(t_settings) / sizeof(flash_addr[0]))) > flash_addr_end)
		{	// no more flash space left to save into, start again by first erasing ALL allocated eeprom flash pages

			// can't erase flash pages without first unlocking the flash (write protection)
			if (HAL_OK != HAL_FLASH_Unlock())
				return -1;        // unlock error :(

			// erase all allocated eeprom flash pages
			const HAL_StatusTypeDef status = HAL_FLASHEx_Erase(&erase_init, &page_error);

			// re-lock the flash (write protection)
			HAL_FLASH_Lock();

			if (status != HAL_OK)
				return -2;        // erase error

			flash_addr = (uint32_t *)EEPROM_START_ADDRESS;   // start writing at the beginning again
		}
	}
	else
	{	// start saving at the beginning of allocated eeprom flash
		flash_addr = (uint32_t *)EEPROM_START_ADDRESS;
	}

	{	// see if the flash slot is empty (or not)
		//
		// we can't write to it if it's not suitably empty
		// the flash bits can be set to '0' without erasing the flash page
		// but can't be set to '1' without first doing a full page flash erase - beware, I think some STM32's are the opposite (if you re-use this code)

		const __IO uint32_t *flash = flash_addr;               // flash address we want to write too
		const uint32_t      *p     = (uint32_t *)&settings;    // address of data we want to write into flash
		unsigned int         i     = 0;

		while (i < sizeof(t_settings))
		{
			const uint32_t set_val = *p++;
			if ((*flash++ & set_val) == set_val)
			{	// this flash word is good for writing into
				i += sizeof(uint32_t);
				continue;
			}

			// can't program this particular flash word without first erasing it.
			//
			// just move too and check the next slot to save into

			flash_addr += sizeof(t_settings) / sizeof(flash_addr[0]);

			flash = flash_addr;               // flash address
			p     = (uint32_t *)&settings;    // system settings address
			i     = 0;

			if ((flash_addr + (sizeof(t_settings) / sizeof(flash_addr[0]))) <= flash_addr_end)
				continue;                     // go back and check the slot

			// no more flash space left to save into, start again by erasing all allocated eeprom flash pages

			if (HAL_OK != HAL_FLASH_Unlock())
				return -3;        // :(
			const HAL_StatusTypeDef status = HAL_FLASHEx_Erase(&erase_init, &page_error);
			HAL_FLASH_Lock();
			if (status != HAL_OK)
				return -4;        // hmmmm

			flash_addr = (uint32_t *)EEPROM_START_ADDRESS;   // flash address to write too
			break;
		}
	}

	// write the new settings into flash
	//
	// the flash address to write too is contained in 'flash_addr'

	if (HAL_OK != HAL_FLASH_Unlock())
		return -5;            // tut :(

	for (unsigned int i = 0; i < sizeof(t_settings); i += sizeof(uint16_t))
	{
		const uint16_t w = ((uint16_t *)&settings)[i / sizeof(uint16_t)];   // data word to write too flash

		// write the word into flash
		const HAL_StatusTypeDef status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, (uint32_t)flash_addr + i, w);
		if (status == HAL_OK)
			continue;           // it worked !

		// no it didunt :(

		if ((uint32_t)flash_addr == EEPROM_START_ADDRESS)
		{	// erase this allocated flash page, then give up
			erase_init.NbPages = 1;
			HAL_FLASHEx_Erase(&erase_init, &page_error);
		}

		HAL_FLASH_Lock();

		return -6;
	}

	HAL_FLASH_Lock();

	{	// confirm the new flash write went OK by reading back and checking for data match

		const uint32_t      *set   = (uint32_t *)&settings;   // point to our system settings
		const __IO uint32_t *flash = flash_addr;              // flash address we wrote them too

		for (unsigned int i = 0; i < sizeof(t_settings); i += sizeof(uint32_t))
		{
			if (*flash++ == *set++)
				continue;           // seems good !

			// no it ain't :(
			//
			// erase this flash page, then go do something else

			erase_init.NbPages = 1;
			if (HAL_OK != HAL_FLASH_Unlock())
				return -7;     // grrr
			HAL_FLASHEx_Erase(&erase_init, &page_error);
			HAL_FLASH_Lock();

			return -8;         // freedom !
		}
	}

	// settings saved OK !

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

void goertzel_wrap(const float *in_samples, t_comp *out_samples, const unsigned int len, const unsigned int g_len, t_goertzel *g)
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
	}
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
// DSP stuff

int unit_conversion(float *value)
{
	if (*value < 1e-12f)
	{	// femto
		*value *= 1e15f;
		return -5;
	}

	if (*value < 1e-9f)
	{	// pico
		*value *= 1e12f;
		return -4;
	}

	if (*value < 1e-6f)
	{	// nano
		*value *= 1e9f;
		return -3;
	}

	if (*value < 1e-3f)
	{	// micro
		*value *= 1e6f;
		return -2;
	}

	if (*value < 1e0f)
	{	// milli
		*value *= 1e3f;
		return -1;
	}

	if (*value < 1e3f)
	{	// unit
		return 0;
	}

	if (*value < 1e6f)
	{	// kilo
		*value *= 1e-3f;
		return 1;
	}

	if (*value < 1e9f)
	{	// Mega
		*value *= 1e-6f;
		return 2;
	}

	// Giga
	*value *= 1e-9f;
	return 3;
}

void generate_ref_signal(const unsigned int length)
{
	// Assume two complete cycles, each divided into 4 quadrants
	// Total quadrants = 8; each quadrant length:

	const unsigned int per_len_quadrant = length / 8;

	for (unsigned int i = 0; i < 8; i++)
	{
		// Use (i % 4) to cycle through quadrants for one complete cycle
		const unsigned int quadrant    = i % 4;
		const unsigned int start_index = i * per_len_quadrant;

		for (unsigned int j = 0; j < per_len_quadrant; j++)
		{
			switch (quadrant)
			{
				case 0: //   0°  -90°: ref0 = +1, ref90 = +1
					ref0_data[ start_index + j] =  1;
					ref90_data[start_index + j] =  1;
					break;

				case 1: //  90° -180°: ref0 = +1, ref90 = -1
					ref0_data[ start_index + j] =  1;
					ref90_data[start_index + j] = -1;
					break;

				case 2: // 180° -270°: ref0 = -1, ref90 = -1
					ref0_data[ start_index + j] = -1;
					ref90_data[start_index + j] = -1;
					break;

				case 3: // 270° -360°: ref0 = -1, ref90 = +1
					ref0_data[ start_index + j] = -1;
					ref90_data[start_index + j] =  1;
					break;
			}
		}
	}
}

void set_measurement_frequency(const uint32_t Hz)
{


	// TODO: make this adjustment automatic by inspecting the sine wave using a histogram of the sampled samples
	//       spikes in the histogram indicate clipping (several similar values)



	if (Hz == 100)
		set_DAC_amplitude(0.52);
	else
	if (Hz == 500)
		set_DAC_amplitude(0.62);
	else
		set_DAC_amplitude(1.0);

	if (Hz > 0)
	{
		const uint32_t timer_rate_Hz = (DMA_ADC_DATA_LENGTH / 2) * Hz;          // for one cycle
		const uint32_t period        = (((HAL_RCC_GetHCLKFreq() / (htim3.Init.Prescaler + 1)) + (timer_rate_Hz / 2)) / timer_rate_Hz) - 1;
		__HAL_TIM_SET_AUTORELOAD(&htim3, period);
	}
}

/*
t_complex <float> serial_to_parallel(t_complex <float> z)
{	// convert serial impedance to parallel impedance equivalent

	const float z_sq_sum = (z.re * z.re) + (z.im * z.im);

	if (z.re != 0 && z.im != 0)
		return t_complex <float> (z_sq_sum / z.re, z_sq_sum / z.im);

	if (z.re != 0 && z_sq_sum > 0)
		return t_complex <float> (z_sq_sum / z.re, _FPCLASS_PINF);

	if (z.re != 0 && z_sq_sum < 0)
		return t_complex <float> (z_sq_sum / z.re, _FPCLASS_NINF);

	if (z.im != 0 && z_sq_sum > 0)
		return t_complex <float> (_FPCLASS_PINF, z_sq_sum / z.re);

	if (z.im != 0 && z_sq_sum < 0)
		return t_complex <float> (_FPCLASS_NINF, z_sq_sum / z.re);

	if (z_sq_sum == 0)
		return t_complex <float> (0, 0);

	return t_complex <float> (_FPCLASS_PINF, _FPCLASS_PINF);
}
*/
/*
float lcr_compute(const unsigned int mode, const uint16_t freq, const float impedance, const float phase)
{
	const float phase_rad = phase * DEG_TO_RAD;
	const float resistive = impedance * fabsf(cosf(phase_rad));
	const float reactance = impedance * fabsf(sinf(phase_rad));

	if (reactance < 1e-6f)
		return 0;

	const float omega = (float)(2.0 * M_PI) * freq;   // angular frequency in rad/s

	switch (mode)
	{
		case LCR_MODE_INDUCTANCE:      // L = X / ω
			return reactance / omega;

		case LCR_MODE_CAPACITANCE:     // C = 1 / (ωX)
			return 1.0f / (omega * reactance);

		case LCR_MODE_RESISTANCE:      // ESR: real part of the impedance
			return resistive;

		case LCR_MODE_TAN_DELTA:       // Tan Delta calculation
			return resistive / reactance;

		default:
			return 0;
	}
}
*/
// compute phase difference between two waveforms using complex samples (ie, from the I/Q outputs of a goertzel dft)
//
float phase_diff(const t_comp c1, const t_comp c2)
{
	t_comp d;

//	const t_comp c1 = goertzel_block(&adc_buffer_1, num_samples);		// waveform 1
//	const t_comp c2 = goertzel_block(&adc_buffer_2, num_samples);		// waveform 2

	// multiply
	d.re = (c1.re * c2.re) + (c1.im * c2.im);
	d.im = (c1.re * c2.im) - (c1.im * c2.re);

	// phase
	const float phase_deg = (d.re != 0.0f) ? fmodf((atan2f(d.im, d.re) * RAD_TO_DEG) + 270, 360) : NAN;

	return phase_deg;
}

float phase_compute(const float in_array[], const unsigned int start_l, const unsigned int length)
{
	// multiply signal by 0° and 90° square wave references
	float I_sum = 0;
	float Q_sum = 0;
	for (unsigned int i = start_l; i < length; i++)
	{
		I_sum += in_array[i] * ref0_data[i];
		Q_sum += in_array[i] * ref90_data[i];
	}

	// normalize by sample count and scale for square-wave demodulation (2 / pi)
	const float scale = (float)(M_PI / 2.0) / (length - start_l);
	const float I_avg = I_sum * scale;
	const float Q_avg = Q_sum * scale;

	// compute phase angle
	const float phase_deg = (I_avg != 0.0f) ? atan2f(Q_avg, I_avg) * RAD_TO_DEG : NAN;

	return phase_deg;
}

void compute_amplitude(void)
{
	float sum_sq_adc[8]  = {0};
	float rms_val_adc[8] = {0};
	//float amp_val_adc[8] = {0};

	const unsigned int n = DMA_ADC_DATA_LENGTH;

	const float avg_scale = 1.0f / n;
//	const float sqrt2     = sqrtf(2.0f);

	// accumulate squared deviations
	for (unsigned int row = 0; row < 8; row++)
	{
		for (unsigned int col = 0; col < n; col++)
		{
			float val_sq = 0;
			if ((row % 2) == 0)
				val_sq = adc_data[row][col];              // ERROR ?????
			else
				val_sq = adc_data[row][col];              // ERROR ?????

			sum_sq_adc[row] += val_sq * val_sq;
		}

		// compute RMS
		rms_val_adc[row] = sqrtf(sum_sq_adc[row] * avg_scale);

		// peak amplitude = RMS * sqrtf(2)
		//amp_val_adc[row] = rms_val_adc[row] * sqrt2;
	}

	// Automatic Gain Selection
	// 1. Set Default as Gain A
	volt_gain_sel = 0;
	amp_gain_sel  = 0;

	// 2. Check the gain value
	const float threshold = 50;
	if (rms_val_adc[(volt_gain_sel * 4) + 0] < threshold)
		volt_gain_sel = 1;
	if (rms_val_adc[(amp_gain_sel  * 4) + 2] < threshold)
		amp_gain_sel = 1;

	// Update system_data struct with amplitude after conversion
	system_data.rms_voltage     = adc_to_volts(rms_val_adc[(volt_gain_sel * 4) + 0]);
	system_data.rms_afc_volt    = adc_to_volts(rms_val_adc[(volt_gain_sel * 4) + 1]);
	system_data.rms_current     = adc_to_volts(rms_val_adc[(amp_gain_sel  * 4) + 2]);
	system_data.rms_afc_current = adc_to_volts(rms_val_adc[(amp_gain_sel  * 4) + 3]);
}

// pass the new ADC samples through the goertzel dft
//
// goertzel output samples are 100% free of any DC offset, also very much cleaned of any out-of-band noise (crucial for following measurements)
//
// input is real
// output is complex (I/Q)
//
void process_Goertzel(void)
{
	// the Goertzel DFT is used for creating I/Q output, phase computation and filtering (if desired)

	// the STM32F103 MCU doesn't have a HW FPU, so full length filtering takes quite a time :(
	//
	// a better (drop in replacement) MCU to use would be the STM32F303CBT6 (or others), it has a HW FPU which greatly improves FP ops
	// it also has a dual 12-DAC, but the two pins it outputs on can't be used for DAC output without modding the m181 PCB
	// even so, the FPU alone would be a huge gain in speeding up the FP computation frame rate
	//
	// STM32F103CBT6 drop-in replacements .. STM32F303CBT6, STM32L412CBT6, STM32L431CCT6 and STM32L433CBT6

	const float avg_coeff = (adc_avg_count >= 5) ? 0.5 : 0.9;

	for (unsigned int buf_index = 0; buf_index < ARRAY_SIZE(adc_data); buf_index++)
	{
		#if !defined(GOERTZEL_FILTER_LENGTH) || (GOERTZEL_FILTER_LENGTH <= 0)
			uint8_t filter = 0;
		#else
			uint8_t filter = 1;
		#endif

		if (!settings.uart_all_print_dso)
			filter = 0;      // no need to fully filter if not sending the waveforms down the serial port

		if (!filter)
		{	// don't filter the waveform, do these ..
			//    remove waveform DC offset
			//   compute waveform RMS magnitude
			//   compute waveform phase

			register float *buf = adc_data[buf_index];   // point to the ADC samples

			{	// remove waveform DC offset

				// compute the average (DC offset)
				register float sum = 0;
				for (unsigned int k = 0; k < DMA_ADC_DATA_LENGTH; k++)
					sum += buf[k];
				sum *= 1.0f / DMA_ADC_DATA_LENGTH;

				#if 1
				{	// update a rolling average value (not really needed but hey ho)
					adc_avg[buf_index].re = ((1.0f - avg_coeff) * adc_avg[buf_index].re) + (avg_coeff * sum);
					sum = adc_avg[buf_index].re;
				}
				#endif

				// remove DC offset using computed average (DC offset)
				for (unsigned int k = 0; k < DMA_ADC_DATA_LENGTH; k++)
					buf[k] -= sum;
			}

			{	// compute waveform RMS magnitude

				register float sum = 0;
				for (unsigned int k = 0; k < DMA_ADC_DATA_LENGTH; k++)
					sum += SQR(buf[k]);
				sum *= 1.0f / DMA_ADC_DATA_LENGTH;

				mag_rms[buf_index] = sqrtf(sum);
			}

			{	// compute waveform phase
				goertzel_block(buf, DMA_ADC_DATA_LENGTH, &goertzel);
				phase_deg[buf_index] = (goertzel.re != 0.0f) ? fmodf((atan2f(goertzel.im, goertzel.re) * RAD_TO_DEG) + 270, 360) : NAN;
			}
		}
		else
		{	// use Goertzel dft to filter the waveforms, length of filter is settable

			const unsigned int filter_len = GOERTZEL_FILTER_LENGTH;

			{	// compute waveform phase
				goertzel_block(adc_data[buf_index], DMA_ADC_DATA_LENGTH, &goertzel);
				phase_deg[buf_index] = (goertzel.re != 0.0f) ? fmodf((atan2f(goertzel.im, goertzel.re) * RAD_TO_DEG) + 270, 360) : NAN;
			}

			register t_comp *buf = tmp_buf;            // point to Goertzel dft output samples

			goertzel_wrap(adc_data[buf_index], buf, DMA_ADC_DATA_LENGTH, filter_len, &goertzel);

			if (filter_len != (DMA_ADC_DATA_LENGTH / 2) && filter_len != DMA_ADC_DATA_LENGTH)
			{	// need to remove DC offset because the Goertzel filter is not a multiple number of sine cycles in length

				// compute the average (DC offset)
				register t_comp sum = {0, 0};
				for (unsigned int k = 0; k < DMA_ADC_DATA_LENGTH; k++)
				{
					register const t_comp samp = buf[k];
					sum.re += samp.re;
					sum.im += samp.im;
				}
				sum.re *= 1.0f / DMA_ADC_DATA_LENGTH;
				sum.im *= 1.0f / DMA_ADC_DATA_LENGTH;

				#if 1
				{	// update a rolling average value (not really needed but hey ho)
					adc_avg[buf_index].re = ((1.0f - avg_coeff) * adc_avg[buf_index].re) + (avg_coeff * sum.re);
					adc_avg[buf_index].im = ((1.0f - avg_coeff) * adc_avg[buf_index].im) + (avg_coeff * sum.im);
					sum.re = adc_avg[buf_index].re;
					sum.im = adc_avg[buf_index].im;
				}
				#endif

				// remove DC offset using computed average (DC offset)
				for (unsigned int k = 0; k < DMA_ADC_DATA_LENGTH; k++)
				{
					buf[k].re -= sum.re;
					buf[k].im -= sum.im;
				}
			}

			{	// compute RMS magnitude and save the Goertzel filtered output samples

				register float *buf_out = adc_data[buf_index];

				register float sum = 0;
				for (unsigned int k = 0; k < DMA_ADC_DATA_LENGTH; k++)
				{
					register const t_comp samp = buf[k];

					sum += SQR(samp.re) + SQR(samp.im);

					// '#if 0' to test without goertzel dft filtering
					#if 1
						// replace the unfiltered samples with the filtered samples
						buf_out[k] = samp.re;      // for now, just keep the real (0 deg) output (imag/90-deg is dropped :( )
					#endif
				}
				sum *= 1.0f / DMA_ADC_DATA_LENGTH;

				// save the RMS magnitude
				mag_rms[buf_index] = sqrtf(sum);
			}
		}
	}

	adc_avg_count++;
}

void combine_afc(const unsigned int vi, float *avg_mag_rms, float *avg_deg)
{	// combine AFC mag/phase results (the AFC sample blocks are all the same so use the average)
	//
	// voltage results .. vi = 0
	// current results .. vi = 1

	unsigned int sum_count    = 0;
	float        sum_mag_rms  = 0;
	t_comp       sum_phase    = {0, 0};

	// scan over each AFC results
	for (unsigned int i = 1; i < 4; i += 2)
	{
		const unsigned int buf_index = ((vi & 1u) * 4) + i;

		sum_mag_rms += mag_rms[buf_index];

		const float phase_rad = phase_deg[buf_index] * DEG_TO_RAD;
		sum_phase.re += cosf(phase_rad);
		sum_phase.im += sinf(phase_rad);

		sum_count++;
	}

	*avg_mag_rms = sum_mag_rms / sum_count;
	*avg_deg     = (sum_phase.re != 0.0f) ? atan2f(sum_phase.im, sum_phase.re) * RAD_TO_DEG : NAN;
}

void process_data(void)
{
	float voltage_afc_mag_rms = 0;
	float voltage_afc_deg     = 0;
	float current_afc_mag_rms = 0;
	float current_afc_deg     = 0;

	process_Goertzel();

	combine_afc(0, &voltage_afc_mag_rms, &voltage_afc_deg);
	combine_afc(1, &current_afc_mag_rms, &current_afc_deg);

	// *********************
	// automatic gain selection

	const float threshold = 50;
	volt_gain_sel = 0;
	amp_gain_sel  = 0;
	volt_gain_sel = (mag_rms[(volt_gain_sel * 4) + 0] <= threshold) ? 1 : 0;
	amp_gain_sel  = (mag_rms[(volt_gain_sel * 4) + 2] <= threshold) ? 1 : 0;


	// amplitude after conversion
	system_data.rms_voltage     = adc_to_volts(mag_rms[(volt_gain_sel * 4) + 0]);
	system_data.rms_afc_volt    = adc_to_volts(mag_rms[(volt_gain_sel * 4) + 1]);
	system_data.rms_current     = adc_to_volts(mag_rms[(amp_gain_sel  * 4) + 2]);
	system_data.rms_afc_current = adc_to_volts(mag_rms[(amp_gain_sel  * 4) + 3]);

	// *********************

//	compute_amplitude();

	system_data.impedance = system_data.rms_voltage / system_data.rms_current;

	// **************************
	// applying the correction Factor - manually
	//
	// TODO: automatically (calibrate) measure the opamp gain on each of the two gain paths

	system_data.impedance = volt_gain_sel ? system_data.impedance /  101           : system_data.impedance;
	system_data.impedance = amp_gain_sel  ? system_data.impedance * (101 * 1.017f) : system_data.impedance * 0.874f;

	// **************************

	phase_offset_array_index = 0;

	system_data.voltage_phase  =       phase_compute(adc_data[(amp_gain_sel * 4) + 0], phase_offset_array_index, DMA_ADC_DATA_LENGTH) -
	                                   phase_compute(adc_data[(amp_gain_sel * 4) + 2], phase_offset_array_index, DMA_ADC_DATA_LENGTH);

	system_data.current_phase  = fabsf(phase_compute(adc_data[(amp_gain_sel * 4) + 2], phase_offset_array_index, DMA_ADC_DATA_LENGTH) -
	                                   phase_compute(adc_data[(amp_gain_sel * 4) + 3], phase_offset_array_index, DMA_ADC_DATA_LENGTH));
	system_data.current_phase -= 180.0f;

	system_data.vi_phase       = fabsf(fabsf(system_data.voltage_phase) - fabsf(system_data.current_phase));

	const float omega      = (float)(2.0 * M_PI) * settings.measuring_Hz;   // angular frequency in rad/s
	const float phase_rad  = system_data.vi_phase * DEG_TO_RAD;
	const float resistive  = system_data.impedance * fabsf(cosf(phase_rad));
	const float reactance  = system_data.impedance * fabsf(sinf(phase_rad));
//	if (reactance < 1e-6f)
//		return 0;
	const float inductance  = reactance / omega;                            // L = X / ω
	const float capacitance = 1.0f / (omega * reactance);                   // C = 1 / (ωX)
	const float esr         = resistive;
	const float tan_delta   = resistive / reactance;

	system_data.inductance  = inductance;
	system_data.capacitance = capacitance;
	system_data.resistance  = system_data.impedance;
	system_data.esr         = esr;
	system_data.tan_delta   = tan_delta;

	// Unit conversion for capacitance, inductance, and resistance
	system_data.unit_capacitance = unit_conversion(&system_data.capacitance);
	system_data.unit_inductance  = unit_conversion(&system_data.inductance);
	system_data.unit_resistance  = unit_conversion(&system_data.resistance);
	system_data.unit_esr         = unit_conversion(&system_data.esr);
}

void process_ADC(const void *buffer)
{
	// process the new ADC 12-bit samples

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

	if (vi_index == 0 && adc_data_avg_count == 0)
	{	// the first sample block after setting the HW mode pins

		set_measure_mode_pins(vi_mode);                                     // ensure the HW mode pins are set correctly

		HAL_GPIO_WritePin(LED_pin_GPIO_Port, LED_Pin, GPIO_PIN_SET);        // TEST only, LED on
	}

	// each time the HW VI mode is changed, the ADC input sees a large unwanted spike/DC-offset that takes time to settle :(
	// so we simply discard a number of sample blocks after each HW VI mode change
	//
	// actually, the spike only appears to occur after the GS pin (gain setting) is changed

	// skip less blocks if the gain pin hasn't changed
	const unsigned int skip_block_count = ((vi_mode >> 1) == (prev_vi_mode >> 1)) ? MODE_SWITCH_BLOCK_WAIT_SHORT : MODE_SWITCH_BLOCK_WAIT_LONG;

	if (adc_data_avg_count >= skip_block_count)
	{	// add the new sample block to the averaging buffer
		for (unsigned int i = 0; i < DMA_ADC_DATA_LENGTH; i++)
		{
			adc_data_avg[i].adc += adc_buffer[i].adc - 2048;
			adc_data_avg[i].afc += adc_buffer[i].afc - 2048;
		}
	}

	if (++adc_data_avg_count < (skip_block_count + adc_average_count))
		return;

	// next measurement mode
	vi_index++;

	// set the GS/VI pins ready for the next measurement run
	set_measure_mode_pins(vi_measure_mode_table[vi_index]);

	{	// fetch/scale the averaged ADC samples
		register const float scale = 1.0f / (adc_data_avg_count - skip_block_count);

		// buffers to save the new data into
		register float *buf_adc = adc_data[buf_index + 0];
		register float *buf_afc = adc_data[buf_index + 1];

		for (unsigned int i = 0; i < DMA_ADC_DATA_LENGTH; i++)
		{
			buf_adc[i] = adc_data_avg[i].adc * scale;        // averaged sample (saved as float)
			buf_afc[i] = adc_data_avg[i].afc * scale;        //    "      "
		}
	}

	// reset averaging buffer ready for the next measurement run
	memset(adc_data_avg, 0, sizeof(adc_data_avg));
	adc_data_avg_count = 0;

	// remember current VI mode
	prev_vi_mode = vi_mode;

	// save the new VI mode for the next measurement run
	vi_measure_index = vi_index;

	if (vi_index >= VI_MODE_DONE)
		HAL_GPIO_WritePin(LED_pin_GPIO_Port, LED_Pin, GPIO_PIN_RESET);    // TEST only, LED off
}

// ***********************************************************

// available font sizes ..
//  Font_7x10
//  Font_11x18
//  Font_16x26
//
// Font_11x18 .. big
// Font_7x10  .. small general

void print_sprint(const unsigned int digit, const float value, char *output_char)
{
    if (digit == 4)
    {
        if (value < 10.0f)
            sprintf(output_char, "%.3f", value); // 1.234
        else
		if (value < 100.0f)
            sprintf(output_char, "%.2f", value); // 12.34
        else
		if (value < 1000.0f)
            sprintf(output_char, "%.1f", value); // 123.4
        else
            sprintf(output_char, "%.0f", value); // 1234 (no dp)
    }
    else
	if (digit == 2)
    {
        if (value < 10.0f)
            sprintf(output_char, "%.1f'", value); // 1.2
        else
            sprintf(output_char, "%.0f'", value); // 12 (no dp)
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
	ssd1306_SetCursor(0, 0);
	ssd1306_WriteString("M181", Font_7x10, White);

	ssd1306_SetCursor(90, 0);
	sprintf(buffer_display, "v%.2f", FW_VERSION);
	ssd1306_WriteString(buffer_display, Font_7x10, White);

	ssd1306_SetCursor(16, 14);
	ssd1306_WriteString("LCR Meter", Font_11x18, White);

	#if 0
		ssd1306_FillRectangle(10, 32 - 1, SSD1306_WIDTH - 10, 32, White);
	#elif 0
		// dotted line
		for (unsigned int x = 10; x < (SSD1306_WIDTH - 10); x += 2)
			ssd1306_DrawPixel(x, 32 - 1, White);
	#endif

	ssd1306_SetCursor(5, 38);
	ssd1306_WriteString("HW by JYETech", Font_7x10, White);
	ssd1306_SetCursor(5, 52);
	ssd1306_WriteString("FW by Jaishankar", Font_7x10, White);

	ssd1306_UpdateScreen();
}

void draw_screen(const uint8_t full_update)
{
	//#define DRAW_LINES          // use this if you want horizontal lines drawn

	const uint8_t offset_x       = 1;
	const uint8_t line_spacing_y = 13;

	const uint8_t line1_y = 0;
	const uint8_t val1_x  = offset_x;
	const uint8_t val2_x  = 36;
	const uint8_t val3_x  = 59;
	const uint8_t val4_x  = 92;

	#ifdef DRAW_LINES
		const uint8_t line2_y = line_spacing_y + 4;  // with horizontal lines
	#else
		const uint8_t line2_y = line_spacing_y + 3;  // no horizontal lines
	#endif
	uint8_t       val21_x = offset_x;
	const uint8_t val22_x = 22;
	const uint8_t val23_x = 79;
	const uint8_t val24_x = 104;

	#ifdef DRAW_LINES
		const uint8_t line3_y = 10 + line2_y + 0 + line_spacing_y;  // with horizontal lines
	#else
		const uint8_t line3_y = 10 + line2_y + 1 + line_spacing_y;  // no horizontal lines
	#endif
	uint8_t       val31_x = offset_x;
	const uint8_t val32_x = 20;
	const uint8_t val33_x = 62;
	const uint8_t val34_x = 75;
	const uint8_t val35_x = 116;

	const uint8_t line4_y = line3_y + line_spacing_y;
	const uint8_t val41_x = offset_x;
	const uint8_t val42_x = 20;
	const uint8_t val43_x = 62;
	const uint8_t val44_x = 75;
	const uint8_t val45_x = 116;

	if (draw_screen_count == 0 || full_update)
	{	// full redraw

		// clear the screen
		ssd1306_Fill(Black);

		// Line 1

		ssd1306_SetCursor(val1_x, line1_y);
		ssd1306_WriteString("SER", Font_7x10, White);

		ssd1306_SetCursor(val3_x, line1_y);
		ssd1306_WriteString("kHz", Font_7x10, White);

		ssd1306_SetCursor(val4_x, line1_y);
		sprintf(buffer_display, "v%.2f", FW_VERSION);
		ssd1306_WriteString(buffer_display, Font_7x10, White);

		// Line 2

		switch (op_mode)
		{
			default:
			case OP_MODE_MEASURING:
			{
				// horizontal line(s)
				#ifdef DRAW_LINES
					#if 0
						// solid line
						ssd1306_FillRectangle(0, line_spacing_y - 1, SSD1306_WIDTH, line_spacing_y, White);
					#else
						// dotted lines
						for (unsigned int x = 0; x < SSD1306_WIDTH; x += 3)
						{
							ssd1306_DrawPixel(x, line_spacing_y - 1, White);
							ssd1306_DrawPixel(x, line3_y - 4, White);
						}
					#endif
				#endif

				// Line 3

				ssd1306_SetCursor(val31_x, line3_y);
				ssd1306_WriteString("V  ", Font_7x10, White);

				ssd1306_SetCursor(val33_x, line3_y);
				ssd1306_WriteString("A ", Font_7x10, White);

				// Line 4

				switch (settings.lcr_mode)
				{
					case LCR_MODE_INDUCTANCE:
					case LCR_MODE_CAPACITANCE:
						ssd1306_SetCursor(val41_x, line4_y);
						ssd1306_WriteString("ER ", Font_7x10, White);

						ssd1306_SetCursor(val43_x, line4_y);
						ssd1306_WriteString("D ", Font_7x10, White);
						break;

					case LCR_MODE_RESISTANCE:
						break;
				}
			}

			case OP_MODE_OPEN_PROBE_CALIBRATION:
			{

				break;
			}

			case OP_MODE_SHORTED_PROBE_CALIBRATION:
			{

				break;
			}
		}

		draw_screen_count = 0;
	}

	if (vi_measure_index >= VI_MODE_DONE || full_update)
	{
		// Line 1: Frequency display

		ssd1306_SetCursor(val2_x, line1_y);
		sprintf(buffer_display, "%0.1f", settings.measuring_Hz * 1e-3f);  // kHz
		ssd1306_WriteString(buffer_display, Font_7x10, White);

		// Line 2: Mode

		switch (op_mode)
		{
			default:
			case OP_MODE_MEASURING:
				{
					float value = 0;

					switch (settings.lcr_mode)
					{
						case LCR_MODE_INDUCTANCE:
							sprintf(buffer_display, "L ");
							value = system_data.inductance;
							break;

						case LCR_MODE_CAPACITANCE:
							sprintf(buffer_display, "C ");
							value = system_data.capacitance;
							break;

						case LCR_MODE_RESISTANCE:
							sprintf(buffer_display, "R ");
							value = system_data.resistance;
							break;
					}
					ssd1306_SetCursor(val21_x, line2_y);
					ssd1306_WriteString(buffer_display, Font_11x18, White);

					// Line 2: unit display

					switch (settings.lcr_mode)
					{
						case LCR_MODE_INDUCTANCE:
/*							if (system_data.unit_inductance <= -3)
								sprintf(buffer_display, "nH");
							else
							if (system_data.unit_inductance == -2)
								sprintf(buffer_display, "uH");
							else
							if (system_data.unit_inductance == -1)
								sprintf(buffer_display, "mH");
							else
								sprintf(buffer_display, "H ");
*/
							sprintf(buffer_display, "%2d", system_data.unit_inductance);  // TEST
							break;

						case LCR_MODE_CAPACITANCE:
/*							if (system_data.unit_capacitance <= -4)
								sprintf(buffer_display, "pF");
							else
							if (system_data.unit_capacitance == -3)
								sprintf(buffer_display, "nF");
							else
							if (system_data.unit_capacitance == -2)
								sprintf(buffer_display, "uF");
							else
							if (system_data.unit_capacitance == -1)
								sprintf(buffer_display, "mF");
							else
								sprintf(buffer_display, "F ");
*/
							sprintf(buffer_display, "%2d", system_data.unit_capacitance);  // TEST
							break;

						case LCR_MODE_RESISTANCE:
/*							if (system_data.unit_resistance <= -1)
								sprintf(buffer_display, "m");
							else
							if (system_data.unit_resistance == 0)
								sprintf(buffer_display, " ");
							else
							if (system_data.unit_resistance == 1)
								sprintf(buffer_display, "k");
							else
							if (system_data.unit_resistance == 2)
								sprintf(buffer_display, "M");
							else
								sprintf(buffer_display, "G");
							print_custom_symbol(val23_x + 7, line2_y + 4, omega_7x10, 7, 10);
*/
							sprintf(buffer_display, "%2d", system_data.unit_resistance);  // TEST
							break;
					}
					ssd1306_SetCursor(val23_x, line2_y + 4);
					ssd1306_WriteString(buffer_display, Font_7x10, White);

					ssd1306_SetCursor(val22_x, line2_y);
					print_sprint(4, value, buffer_display);
					ssd1306_WriteString(buffer_display, Font_11x18, White);

					ssd1306_SetCursor(val24_x, line2_y + 4);
					print_sprint(4, system_data.vi_phase, buffer_display);
					ssd1306_WriteString(buffer_display, Font_7x10, White);

					// Line 3: Voltage and Current reading

					ssd1306_SetCursor(val32_x, line3_y);
					sprintf(buffer_display, "%.3f", (system_data.rms_voltage >= 0) ? system_data.rms_voltage : 0);
					ssd1306_WriteString(buffer_display, Font_7x10, White);

					ssd1306_SetCursor(val34_x, line3_y);
					sprintf(buffer_display, "%.3f", (system_data.rms_current >= 0) ? system_data.rms_current : 0);
					ssd1306_WriteString(buffer_display, Font_7x10, White);

					switch (settings.lcr_mode)
					{
						case LCR_MODE_INDUCTANCE:
						case LCR_MODE_CAPACITANCE:
							// Line 4: ESR and Tan Delta

							ssd1306_SetCursor(val42_x, line4_y);
							print_sprint(4, system_data.esr, buffer_display);
							ssd1306_WriteString(buffer_display, Font_7x10, White);

							ssd1306_SetCursor(val44_x, line4_y);
							print_sprint(4, system_data.tan_delta, buffer_display);
							ssd1306_WriteString(buffer_display, Font_7x10, White);

							break;

						case LCR_MODE_RESISTANCE:
							break;
					}
				}

				break;

			case OP_MODE_OPEN_PROBE_CALIBRATION:
				sprintf(buffer_display, "OPEN cal %2d", CALIBRATE_COUNT - calibrate.count - 1);
				ssd1306_SetCursor(val21_x, line2_y);
				ssd1306_WriteString(buffer_display, Font_7x10, White);


				break;

			case OP_MODE_SHORTED_PROBE_CALIBRATION:
				sprintf(buffer_display, "SHORTED cal %2d", CALIBRATE_COUNT - calibrate.count - 1);
				ssd1306_SetCursor(val21_x, line2_y);
				ssd1306_WriteString(buffer_display, Font_7x10, White);


				break;
		}

		// Line 4: UART Mode

		ssd1306_SetCursor(val45_x, line4_y);
		sprintf(buffer_display, settings.uart_all_print_dso ? "U" : " ");       // ON/OFF
		ssd1306_WriteString(buffer_display, Font_7x10, White);
	}

	// Line 3: status

	ssd1306_SetCursor(val35_x, line3_y);
	sprintf(buffer_display, "%u", system_data.vi_measure_mode);
	ssd1306_WriteString(buffer_display, Font_7x10, White);

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

void Error_Handler(void)
{
	__disable_irq();
	HAL_GPIO_WritePin(LED_pin_GPIO_Port, LED_Pin, GPIO_PIN_SET);	// LED on
	while (1)
	{
	}
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
	__disable_irq();
	while (1)
	{
		HAL_GPIO_WritePin(LED_pin_GPIO_Port, LED_Pin, GPIO_PIN_SET);	// LED on
		DWT_Delay_us(70000);
		HAL_GPIO_WritePin(LED_pin_GPIO_Port, LED_Pin, GPIO_PIN_RESET);	// LED off
		DWT_Delay_us(70000);
	}
}

void MemManage_Handler(void)
{
	__disable_irq();
	while (1)
	{
		HAL_GPIO_WritePin(LED_pin_GPIO_Port, LED_Pin, GPIO_PIN_SET);	// LED on
		DWT_Delay_us(100000);
		HAL_GPIO_WritePin(LED_pin_GPIO_Port, LED_Pin, GPIO_PIN_RESET);	// LED off
		DWT_Delay_us(100000);
	}
}

void BusFault_Handler(void)
{
	__disable_irq();
	while (1)
	{
		HAL_GPIO_WritePin(LED_pin_GPIO_Port, LED_Pin, GPIO_PIN_SET);	// LED on
		DWT_Delay_us(100000);
		HAL_GPIO_WritePin(LED_pin_GPIO_Port, LED_Pin, GPIO_PIN_RESET);	// LED off
		DWT_Delay_us(100000);
	}
}

void UsageFault_Handler(void)
{
	__disable_irq();
	while (1)
	{
		HAL_GPIO_WritePin(LED_pin_GPIO_Port, LED_Pin, GPIO_PIN_SET);	// LED on
		DWT_Delay_us(100000);
		HAL_GPIO_WritePin(LED_pin_GPIO_Port, LED_Pin, GPIO_PIN_RESET);	// LED off
		DWT_Delay_us(100000);
	}
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

		const int16_t debounce_ms = 50;

		for (unsigned int i = 0; i < ARRAY_SIZE(button); i++)
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
				butt->released     = 0;                             // clear released flag
				butt->held_ms      = 0;                             // reset held down time
				butt->pressed_tick = tick;                          // remember the tick the button was pressed
			}

			if (pressed_tick > 0 && debounce <= 0)
			{	// just released
				butt->held_ms      = tick - pressed_tick;           // save the time the button was held down for
				butt->released     = 1;                             // set released flag
				butt->pressed_tick = 0;                             // reset pressed tick
			}

			if (pressed_tick > 0)
				butt->held_ms      = tick - pressed_tick;           // time the button has been held down for
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
		process_ADC(&raw_adc_dma_data[0]);  // lower half of ADC buffer
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
	if (hadc->Instance == ADC1)
		process_ADC(&raw_adc_dma_data[1]);  // upper half of ADC buffer
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

	const uint32_t timer_rate_Hz = (DMA_ADC_DATA_LENGTH / 2) * 1000;      // 1kHz

	htim3.Instance               = TIM3;
	htim3.Init.CounterMode       = TIM_COUNTERMODE_UP;
	htim3.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
	htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
	htim3.Init.Prescaler         = 0;
	htim3.Init.Period            = (((HAL_RCC_GetHCLKFreq() / (htim3.Init.Prescaler + 1)) + (timer_rate_Hz / 2)) / timer_rate_Hz) - 1;
	if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
	{
		Error_Handler();
	}

	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
	{
		Error_Handler();
	}

	sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
	sMasterConfig.MasterSlaveMode     = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
	{
		Error_Handler();
	}
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
/*
	const uint32_t tick = HAL_GetTick();

	if (button[1].pressed_tick > 0 && (tick - button[1].pressed_tick) >= 500)
	{	// don't wait for button release
		reboot();
		return;
	}
*/

	if (button[0].released)
	{	// HOLD button
		button[0].released     = 0;
		button[0].pressed_tick = 0;

		if (button[0].held_ms >= 800)
		{
			memset(&calibrate, 0, sizeof(calibrate));
			op_mode = OP_MODE_OPEN_PROBE_CALIBRATION;
		}
		else
		{


			// TODO: display HOLD


			// toggle UART data
			settings.uart_all_print_dso = (1u - settings.uart_all_print_dso) & 1u;

		}

		draw_screen(1);
	}

	if (button[1].released)
	{	// S/P button
		button[1].released     = 0;
		button[1].pressed_tick = 0;

		if (button[1].held_ms >= 800)
		{
//			reboot();

			memset(&calibrate, 0, sizeof(calibrate));
			op_mode = OP_MODE_SHORTED_PROBE_CALIBRATION;
		}
		else
		{


			// TODO: Serial/Parallel


			// cycle the frequency
			switch (settings.measuring_Hz)
			{
				case 100:
					settings.measuring_Hz = 500;
					break;
				default:
				case 500:
					settings.measuring_Hz = 1000;
					break;
				case 1000:
					settings.measuring_Hz = 100;
					break;
			}

			set_measurement_frequency(settings.measuring_Hz);

			// save settings
			save_settings_timer = SAVE_SETTINGS_MS;
		}

		draw_screen(1);
	}

	if (button[2].released)
	{	// S/P button
		button[2].released     = 0;
		button[2].pressed_tick = 0;

		if (button[2].held_ms >= 800)
		{

		}
		else
		{
			// cycle through the LCR modes
			unsigned int mode = settings.lcr_mode;
			if (++mode > LCR_MODE_RESISTANCE)
				mode = LCR_MODE_INDUCTANCE;
			settings.lcr_mode = mode;

			// save settings
			save_settings_timer = SAVE_SETTINGS_MS;
		}

		draw_screen(1);
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
	settings.measuring_Hz           = 1000;
//	settings.lcr_mode           = LCR_MODE_INDUCTANCE;
//	settings.lcr_mode           = LCR_MODE_CAPACITANCE;
	settings.lcr_mode           = LCR_MODE_RESISTANCE;
//	settings.uart_all_print_dso = 0;
	settings.uart_all_print_dso = 1;   // send ADC data via the serial port

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
		const float normalized_freq = 2.0f / DMA_ADC_DATA_LENGTH;  // 2 cycles spanning the sample buffer
		goertzel_init(&goertzel, normalized_freq);
	}

	// fetch saved settings from flash
	read_settings();

	system_data.vi_measure_mode = vi_measure_mode_table[vi_measure_index];
	set_measure_mode_pins(system_data.vi_measure_mode);

	set_measurement_frequency(settings.measuring_Hz);

	generate_ref_signal(DMA_ADC_DATA_LENGTH);

	screen_init();
	bootup_screen();

	printf("\r\nrebooted m181 LCR Meter v%.2f\r\n", FW_VERSION);

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

		const unsigned int prev_vi_measure_mode = system_data.vi_measure_mode;
		system_data.vi_measure_mode = vi_measure_mode_table[vi_measure_index];

		if (vi_measure_index >= VI_MODE_DONE)
		{	// full measurement cycle is complete

			// process the new sampled data
			process_data();

			// show it on screen
			draw_screen(0);

			if (settings.uart_all_print_dso)
			{
				const HAL_UART_StateTypeDef state = HAL_UART_GetState(&huart1);
				if (state == HAL_UART_STATE_READY)
				{	// the UART is available to use
					//
					// send the sampled data down the serial link

					#ifdef MATLAB_SERIAL
						// send as ASCII

						const unsigned int cols = 8;
						for (unsigned int i = 0; i < DMA_ADC_DATA_LENGTH; i++)
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

						tx_packet.crc = CRC16_block(0, tx_packet.data, sizeof(tx_packet.data));   // packet CRC - compute the CRC of the data

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
						//memset(&settings.open_probe_calibration, 0, sizeof(settings.open_probe_calibration));

						for (unsigned int i = 0; i < ARRAY_SIZE(calibrate.mag_sum); i++)
							settings.open_probe_calibration.mag_rms[i] = calibrate.mag_sum[i] / calibrate.count;

						for (unsigned int i = 0; i < ARRAY_SIZE(calibrate.phase_sum); i++)
							settings.open_probe_calibration.phase_deg[i] = (calibrate.phase_sum[i].re != 0.0f) ? atan2f(calibrate.phase_sum[i].im, calibrate.phase_sum[i].re) * RAD_TO_DEG : NAN;

						settings.open_probe_calibration.done = 1;

						op_mode = OP_MODE_MEASURING;

						draw_screen(1);
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
						//memset(&settings.shorted_probe_calibration, 0, sizeof(settings.short_probe_calibratiomn);

						for (unsigned int i = 0; i < ARRAY_SIZE(calibrate.mag_sum); i++)
							settings.shorted_probe_calibration.mag_rms[i] = calibrate.mag_sum[i] / calibrate.count;

						for (unsigned int i = 0; i < ARRAY_SIZE(calibrate.phase_sum); i++)
							settings.shorted_probe_calibration.phase_deg[i] = (calibrate.phase_sum[i].re != 0.0f) ? atan2f(calibrate.phase_sum[i].im, calibrate.phase_sum[i].re) * RAD_TO_DEG : NAN;

						settings.shorted_probe_calibration.done = 1;

						op_mode = OP_MODE_MEASURING;

						draw_screen(1);
					}

					break;
				}
			}

			// start next data capture
			vi_measure_index = 0;
		}
		else
		if (system_data.vi_measure_mode != prev_vi_measure_mode && draw_screen_count > 0)
			draw_screen(0);

		if (save_settings_timer == 0)
		{
			if (write_settings() < 0)     // save settings to flash
				write_settings();         // failed, have a 2nd go

			save_settings_timer = -1;     // don't try saving again, until the next time
		}

		#ifdef USE_IWDG
			service_IWDG(0);
		#endif
	}
}

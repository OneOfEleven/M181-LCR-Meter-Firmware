
#include "main.h"
#include "delay.h"
#include "stm32_sw_i2c.h"

#define I2C_CLEAR_SDA     HAL_GPIO_WritePin(SW_I2C_SDA_GPIO_Port, SW_I2C_SDA_Pin, GPIO_PIN_RESET);
#define I2C_SET_SDA       HAL_GPIO_WritePin(SW_I2C_SDA_GPIO_Port, SW_I2C_SDA_Pin, GPIO_PIN_SET);

#define I2C_CLEAR_SCL     HAL_GPIO_WritePin(SW_I2C_SCL_GPIO_Port, SW_I2C_SCL_Pin, GPIO_PIN_RESET);
#define I2C_SET_SCL       HAL_GPIO_WritePin(SW_I2C_SCL_GPIO_Port, SW_I2C_SCL_Pin, GPIO_PIN_SET);

//#define I2C_DELAY         DWT_Delay_us(5); // 5 microsecond delay
#define I2C_DELAY         DWT_Delay_ns(100); // 0.1 microsecond delay .. much faster display update

void I2C_bus_init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	HAL_GPIO_WritePin(SW_I2C_SDA_GPIO_Port, SW_I2C_SDA_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(SW_I2C_SCL_GPIO_Port, SW_I2C_SCL_Pin, GPIO_PIN_SET);

	GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_OD;
	GPIO_InitStruct.Pull  = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
	GPIO_InitStruct.Pin   = SW_I2C_SDA_Pin;
	HAL_GPIO_Init(SW_I2C_SDA_GPIO_Port, &GPIO_InitStruct);
	GPIO_InitStruct.Pin   = SW_I2C_SCL_Pin;
	HAL_GPIO_Init(SW_I2C_SCL_GPIO_Port, &GPIO_InitStruct);
}

//__STATIC_INLINE uint8_t I2C_read_SDA(void)
__STATIC_FORCEINLINE uint8_t I2C_read_SDA(void)
{
	return (HAL_GPIO_ReadPin(SW_I2C_SDA_GPIO_Port, SW_I2C_SDA_Pin) == GPIO_PIN_SET) ? 1 : 0;
}

void I2C_init(void)
{
	I2C_SET_SDA;
	I2C_SET_SCL;
}

void I2C_start_cond(void)
{
	I2C_SET_SCL
	I2C_SET_SDA
	I2C_DELAY
	I2C_CLEAR_SDA
	I2C_DELAY
	I2C_CLEAR_SCL
	I2C_DELAY
}

void I2C_stop_cond(void)
{
	I2C_CLEAR_SDA
	I2C_DELAY
	I2C_SET_SCL
	I2C_DELAY
	I2C_SET_SDA
	I2C_DELAY
}

void I2C_write_bit(uint8_t b)
{
	if (b > 0)
		I2C_SET_SDA
	else
		I2C_CLEAR_SDA

	I2C_DELAY
	I2C_SET_SCL
	I2C_DELAY
	I2C_CLEAR_SCL
}

uint8_t I2C_read_bit(void)
{
	I2C_SET_SDA
	I2C_DELAY
	I2C_SET_SCL
	I2C_DELAY
	const uint8_t b = I2C_read_SDA();
	I2C_CLEAR_SCL
	return b;
}

bool I2C_write_byte(uint8_t B, bool start, bool stop)
{
	if (start)
		I2C_start_cond();

	for (int i = 0; i < 8; i++)
	{
		I2C_write_bit(B & 0x80); // write the most-significant bit
		B <<= 1;
	}

	const uint8_t ack = I2C_read_bit();

	if (stop)
		I2C_stop_cond();

	return !ack;   // 0-ack, 1-nack
}

uint8_t I2C_read_byte(bool ack, bool stop)
{
	uint8_t B = 0;

	for (int i = 0; i < 8; i++)
	{
		B <<= 1;
		B |= I2C_read_bit();
	}

	if (ack)
		I2C_write_bit(0);
	else
		I2C_write_bit(1);

	if (stop)
		I2C_stop_cond();

	return B;
}

bool I2C_send_byte(uint8_t address, uint8_t data)
{
	//if (I2C_write_byte(address << 1, true, false))   // start, send address, write
	if (I2C_write_byte(address, true, false))          // start, send address, write
	{	// send data, stop
		if (I2C_write_byte(data, false, true))
			return true;
	}

	I2C_stop_cond(); // make sure to impose a stop if NAK'd
	return false;
}

uint8_t I2C_receive_byte(uint8_t address)
{
	if (I2C_write_byte((address << 1) | 0x01, true, false)) // start, send address, read
		return I2C_read_byte(false, true);

	return 0;            // return zero if NAK'd
}

_Bool I2C_send_byte_data(uint8_t address, uint8_t reg, uint8_t data)
{
	//if (I2C_write_byte(address << 1, true, false))   // start, send address, write
	if (I2C_write_byte(address, true, false))
		if (I2C_write_byte(reg, false, false)) // send desired register
			if (I2C_write_byte(data, false, true))
				return true; // send data, stop

	I2C_stop_cond();

	return false;
}

uint8_t I2C_receive_byte_data(uint8_t address, uint8_t reg)
{
	//if (I2C_write_byte(address << 1, true, false))   // start, send address, write
	if (I2C_write_byte(address, true, false))
		if (I2C_write_byte(reg, false, false)) // send desired register
			if (I2C_write_byte((address << 1) | 0x01, true, false)) // start again, send address, read
				return I2C_read_byte(false, true); // read data

	I2C_stop_cond();

	return 0; // return zero if NACKed
}

bool I2C_transmit(uint8_t address, uint8_t data[], uint8_t size)
{
	if (I2C_write_byte(address, true, false)) // first byte
	{
		for (int i = 0; i < size; i++)
		{
			if (i >= (size - 1))
			{
				if (I2C_write_byte(data[i], false, true))
					return true;
			}
			else
			{
				if (!I2C_write_byte(data[i], false, false))
					break; // last byte
			}
		}
	}

	I2C_stop_cond();

	return false;
}

bool I2C_receive(uint8_t address, uint8_t reg[], uint8_t *data, uint8_t reg_size, uint8_t size)
{
	if (I2C_write_byte(address, true, false))
	{
		for (int i = 0; i < reg_size; i++)
			if (!I2C_write_byte(reg[i], false, false))
				break;

		if (I2C_write_byte(address | 0x01, true, false)) // start again, send address, read (LSB signifies R or W)
		{
			for (int j = 0; j < size; j++)
				*data++ = I2C_read_byte(false, false);   // read data

			I2C_stop_cond();

			return true;
		}
	}

	I2C_stop_cond();

	return false;
}

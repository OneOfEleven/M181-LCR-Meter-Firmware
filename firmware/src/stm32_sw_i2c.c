
#include "main.h"
#include "delay.h"
#include "stm32_sw_i2c.h"

#define I2C_CLEAR_SDA     LL_GPIO_ResetOutputPin(SW_I2C_SDA_GPIO_Port, SW_I2C_SDA_Pin);
#define I2C_SET_SDA       LL_GPIO_SetOutputPin(  SW_I2C_SDA_GPIO_Port, SW_I2C_SDA_Pin);

#define I2C_CLEAR_SCL     LL_GPIO_ResetOutputPin(SW_I2C_SCL_GPIO_Port, SW_I2C_SCL_Pin);
#define I2C_SET_SCL       LL_GPIO_SetOutputPin(  SW_I2C_SCL_GPIO_Port, SW_I2C_SCL_Pin);

//#define I2C_DELAY         DWT_Delay_ns(50);
#define I2C_DELAY         __NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();

void I2C_bus_init(void)
{
	LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

	I2C_SET_SDA
	I2C_SET_SCL

	GPIO_InitStruct.Mode       = LL_GPIO_MODE_OUTPUT;
	GPIO_InitStruct.Speed      = LL_GPIO_SPEED_FREQ_HIGH;
	GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_OPENDRAIN;
	GPIO_InitStruct.Pull       = LL_GPIO_PULL_UP;
	GPIO_InitStruct.Pin        = SW_I2C_SDA_Pin;
	LL_GPIO_Init(SW_I2C_SDA_GPIO_Port, &GPIO_InitStruct);


	GPIO_InitStruct.Mode       = LL_GPIO_MODE_OUTPUT;
	GPIO_InitStruct.Speed      = LL_GPIO_SPEED_FREQ_HIGH;
	GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	GPIO_InitStruct.Pin        = SW_I2C_SCL_Pin;
	LL_GPIO_Init(SW_I2C_SCL_GPIO_Port, &GPIO_InitStruct);
}

//__STATIC_INLINE uint8_t I2C_read_SDA(void)
__STATIC_FORCEINLINE uint8_t I2C_read_SDA(void)
{
	return LL_GPIO_IsInputPinSet(SW_I2C_SDA_GPIO_Port, SW_I2C_SDA_Pin);
}

void I2C_init(void)
{
	I2C_SET_SDA
	I2C_SET_SCL
}

void I2C_start_cond(void)
{
	I2C_SET_SCL
	I2C_SET_SDA
	I2C_DELAY
	I2C_DELAY
	I2C_CLEAR_SDA
	I2C_DELAY
	I2C_DELAY
	I2C_CLEAR_SCL
	I2C_DELAY
}

void I2C_stop_cond(void)
{
	I2C_CLEAR_SDA
	I2C_DELAY
	I2C_DELAY
	I2C_SET_SCL
	I2C_DELAY
	I2C_DELAY
	I2C_SET_SDA
	I2C_DELAY
}

void I2C_write_bit(const uint8_t b)
{
	if (b)
		I2C_SET_SDA
	else
		I2C_CLEAR_SDA
	I2C_DELAY
	I2C_SET_SCL
	I2C_DELAY
	I2C_DELAY
	I2C_DELAY
	I2C_DELAY
	I2C_DELAY
	I2C_CLEAR_SCL
}

uint8_t I2C_read_bit(void)
{
	I2C_SET_SDA
	I2C_DELAY
	I2C_DELAY
	I2C_SET_SCL
	I2C_DELAY
	I2C_DELAY
	I2C_DELAY
	const uint8_t b = I2C_read_SDA();
	I2C_CLEAR_SCL
	return b;
}

uint8_t I2C_write_byte(const uint8_t B, const uint8_t start, const uint8_t stop)
{
	if (start)
		I2C_start_cond();

	I2C_write_bit(B & 0b10000000);
	I2C_write_bit(B & 0b01000000);
	I2C_write_bit(B & 0b00100000);
	I2C_write_bit(B & 0b00010000);
	I2C_write_bit(B & 0b00001000);
	I2C_write_bit(B & 0b00000100);
	I2C_write_bit(B & 0b00000010);
	I2C_write_bit(B & 0b00000001);

	const uint8_t ack = I2C_read_bit();

	if (stop)
		I2C_stop_cond();

	return !ack;   // 0-ack, 1-nack
}

uint8_t I2C_read_byte(const uint8_t ack, const uint8_t stop)
{
	register uint8_t B;
	B  = I2C_read_bit() << 7;
	B |= I2C_read_bit() << 6;
	B |= I2C_read_bit() << 5;
	B |= I2C_read_bit() << 4;
	B |= I2C_read_bit() << 3;
	B |= I2C_read_bit() << 2;
	B |= I2C_read_bit() << 1;
	B |= I2C_read_bit();

	I2C_write_bit(ack ? 0 : 1);

	if (stop)
		I2C_stop_cond();

	return B;
}

uint8_t I2C_send_byte(const uint8_t address, const uint8_t data)
{
	//if (I2C_write_byte(address << 1, 1, 0))   // start, send address, write
	if (I2C_write_byte(address, 1, 0))          // start, send address, write
	{	// send data, stop
		if (I2C_write_byte(data, 0, 1))
			return 1;
	}

	I2C_stop_cond(); // make sure to impose a stop if NAK'd
	return 0;
}

uint8_t I2C_receive_byte(const uint8_t address)
{
	if (I2C_write_byte((address << 1) | 0x01, 1, 0)) // start, send address, read
		return I2C_read_byte(0, 1);

	return 0;            // return zero if NAK'd
}

uint8_t I2C_send_byte_data(const uint8_t address, const uint8_t reg, const uint8_t data)
{
	//if (I2C_write_byte(address << 1, 1, 0))   // start, send address, write
	if (I2C_write_byte(address, 1, 0))
		if (I2C_write_byte(reg, 0, 0)) // send desired register
			if (I2C_write_byte(data, 0, 1))
				return 1; // send data, stop

	I2C_stop_cond();

	return 0;
}

uint8_t I2C_receive_byte_data(const uint8_t address, const uint8_t reg)
{
	//if (I2C_write_byte(address << 1, 1, 0))   // start, send address, write
	if (I2C_write_byte(address, 1, 0))
		if (I2C_write_byte(reg, 0, 0))                       // send desired register
			if (I2C_write_byte((address << 1) | 0x01, 1, 0)) // start again, send address, read
				return I2C_read_byte(0, 1);                  // read data

	I2C_stop_cond();

	return 0; // return zero if NACKed
}

uint8_t I2C_transmit(const uint8_t address, const uint8_t data[], const uint8_t size)
{
	if (data == NULL || size == 0)
		return 0;

	if (I2C_write_byte(address, 1, 0)) // first byte
	{
		for (unsigned int i = 0; i < size; i++)
		{
			if (i >= (size - 1))
			{
				if (I2C_write_byte(data[i], 0, 1))
					return 1;
			}
			else
			{
				if (!I2C_write_byte(data[i], 0, 0))
					break; // last byte
			}
		}
	}

	I2C_stop_cond();

	return 0;
}

uint8_t I2C_receive(const uint8_t address, const uint8_t reg[], uint8_t *data, const uint8_t reg_size, const uint8_t size)
{
	if (reg == NULL || data == NULL || reg_size == 0 || size == 0)
		return 0;

	if (I2C_write_byte(address, 1, 0))
	{
		for (unsigned int i = 0; i < reg_size; i++)
			if (!I2C_write_byte(reg[i], 0, 0))
				break;

		if (I2C_write_byte(address | 0x01, 1, 0)) // start again, send address, read (LSB signifies R or W)
		{
			for (unsigned int j = 0; j < size; j++)
				*data++ = I2C_read_byte(0, 0);   // read data

			I2C_stop_cond();
			return 1;
		}
	}

	I2C_stop_cond();

	return 0;
}

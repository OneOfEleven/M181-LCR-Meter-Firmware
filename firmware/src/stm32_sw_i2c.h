
#ifndef __STM32_SW_I2C_H
#define __STM32_SW_I2C_H

#include "main.h"

void    I2C_bus_init(void);
void    I2C_init(void);
void    I2C_start_cond(void);
void    I2C_stop_cond(void);
void    I2C_write_bit(uint8_t b);
uint8_t I2C_read_bit(void);
bool    I2C_write_byte(uint8_t B, bool start, bool stop);
uint8_t I2C_read_byte(bool ack, bool stop);
bool    I2C_send_byte(uint8_t address, uint8_t data);
uint8_t I2C_receive_byte(uint8_t address);
bool    I2C_send_byte_data(uint8_t address, uint8_t reg, uint8_t data);
uint8_t I2C_receive_byte_data(uint8_t address, uint8_t reg);
bool    I2C_transmit(uint8_t address, uint8_t data[], uint8_t size);
bool    I2C_receive(uint8_t address, uint8_t reg[], uint8_t *data, uint8_t reg_size, uint8_t size);

#endif

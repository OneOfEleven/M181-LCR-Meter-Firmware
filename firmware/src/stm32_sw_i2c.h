
#ifndef __STM32_SW_I2C_H
#define __STM32_SW_I2C_H

#include <stdint.h>

void    I2C_bus_init(void);
void    I2C_init(void);
void    I2C_start_cond(void);
void    I2C_stop_cond(void);
void    I2C_write_bit(uint8_t b);
uint8_t I2C_read_bit(void);
uint8_t I2C_write_byte(uint8_t B, const uint8_t start, const uint8_t stop);
uint8_t I2C_read_byte(const uint8_t ack, const uint8_t stop);
uint8_t I2C_send_byte(const uint8_t address, const uint8_t data);
uint8_t I2C_receive_byte(const uint8_t address);
uint8_t I2C_send_byte_data(const uint8_t address, const uint8_t reg, const uint8_t data);
uint8_t I2C_receive_byte_data(const uint8_t address, const uint8_t reg);
uint8_t I2C_transmit(const uint8_t address, const uint8_t data[], const uint8_t size);
uint8_t I2C_receive(const uint8_t address, const uint8_t reg[], uint8_t *data, const uint8_t reg_size, const uint8_t size);

#endif

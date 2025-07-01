#ifndef I2C_H
#define I2C_H
#include "stm32f7xx.h"
#include <stddef.h>

void i2c1_write_byte(uint8_t saddr, uint8_t reg_addr, uint8_t data, size_t num_bytes);
void i2c1_read_byte(uint8_t saddr, uint8_t reg_addr, uint8_t *data, size_t num_bytes);
void i2c1_init();

#endif

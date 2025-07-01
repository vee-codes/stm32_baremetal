/*
 * spi.h
 *
 *  Created on: Jun 10, 2025
 *      Author: tony
 */

#ifndef SPI_H
#define SPI_H
#include "stm32f7xx.h"

// function declarations
void spi_init(SPI_TypeDef *spi_device);
void spi_send_msg(uint8_t reg_addr, uint8_t data);

#endif

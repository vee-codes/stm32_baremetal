/*
 * spi.h
 *
 *  Created on: Jun 10, 2025
 *      Author: tony
 */

#ifndef SPI_H_
#define SPI_H_
#include "stm32f7xx.h"

// function declarations
void spi_init(SPI_TypeDef *spi_device);
void test_leds();
void spi_send_msg(uint8_t reg_addr, uint8_t data);
// GPIO MODER
#define GPIO_MODER_INPUT 0UL //input mode -reset state
#define GPIO_MODER_OUTPUT 1UL // general purpose output mode
#define GPIO_MODER_ALTERNATE 2UL // alternate function mode
#define GPIO_MODER_ANALOG 3UL // Analog Mode
#define AF5 5UL


// MAX7219 Register Addresses
#define MAX7219_DECODE 0X09
#define MAX7219_INTENSITY 0X0A
#define MAX7219_SCAN_LIMIT 0X0B
#define MAX7219_SHUTDOWN 0X0C
#define MAX7219_DISPLAY_TEST 0X0F

typedef struct {
    uint8_t decode_mode;      // 0x01
    uint8_t intensity;        // 0x02
    uint8_t scan_limit;       // 0x03
    uint8_t shutdown;         // 0x04
    uint8_t display_test;     // 0x05
    uint8_t digit_data[8];    // 0x10-0x18 (Digit 0-7)
} MAX7219_config_t;

#endif /* SPI_H_ */

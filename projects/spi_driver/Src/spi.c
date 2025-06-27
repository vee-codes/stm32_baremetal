/*
 * spi.c
 *
 *  Created on: Jun 10, 2025
 *      Author: tony
 */
#include "stm32f7xx.h"
#include "spi.h"

void spi_init(SPI_TypeDef *SPI_device){
    //SPI - NSS, SCK, MISO, MOSI
    // SPI1 - PA4,5,6,7 AF5 -b0101

    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    // Alternate Function Mode
    GPIOA->MODER &= ~(GPIO_MODER_MODER4|GPIO_MODER_MODER5|GPIO_MODER_MODER6|GPIO_MODER_MODER7);
    GPIOA->MODER |= (GPIO_MODER_OUTPUT<<GPIO_MODER_MODER4_Pos|
                     GPIO_MODER_ALTERNATE<<GPIO_MODER_MODER5_Pos|
                     GPIO_MODER_ALTERNATE<<GPIO_MODER_MODER6_Pos|
                     GPIO_MODER_ALTERNATE<<GPIO_MODER_MODER7_Pos);
    GPIOA->AFR[0] &= ~(GPIO_AFRL_AFRL5_Msk | GPIO_AFRL_AFRL6_Msk | GPIO_AFRL_AFRL7_Msk);
    GPIOA->AFR[0] |= (AF5<<GPIO_AFRL_AFRL5_Pos |
                      AF5<<GPIO_AFRL_AFRL6_Pos| AF5<<GPIO_AFRL_AFRL7_Pos);
    GPIOA->OSPEEDR |= (3 << GPIO_OSPEEDR_OSPEEDR5_Pos) |  // High speed
                      (3 << GPIO_OSPEEDR_OSPEEDR7_Pos);
    //SPI 2 and 3 use APB1, all others APB2
    RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;
    // set clock (f/2,4,8...256)
    SPI_device->CR1 |= SPI_CR1_BR_0; // f/4 => 4 MHz
    // set ssm (software controlled CS) - control with SSI bit
    SPI_device->CR1 |= (SPI_CR1_SSM | SPI_CR1_SSI| SPI_CR1_MSTR);
    // set CS (GPIO) line high
    GPIOA->ODR |= (GPIO_ODR_OD4);
    // set data size
    SPI_device->CR2 |= 7U<<SPI_CR2_DS_Pos;
    // enable SPI peripheral
    SPI_device->CR1 |= (SPI_CR1_SPE);
}

void spi_send_msg(uint8_t reg_addr, uint8_t data){
    GPIOA->ODR &= ~(GPIO_ODR_OD4);

    uint8_t tmp[2] = {reg_addr, data};
    for(int i =0;i<2;i++){
        while(!(SPI1->SR & SPI_SR_TXE));
        *((volatile uint8_t *)&SPI1->DR) = tmp[i];

    }
    while(SPI1->SR & SPI_SR_BSY);
    GPIOA->ODR |= (GPIO_ODR_OD4);
}

void test_leds(){
    spi_send_msg(MAX7219_DECODE, 0x0UL);
    spi_send_msg(MAX7219_SCAN_LIMIT, 0x07UL);
    spi_send_msg(MAX7219_INTENSITY, 0x09UL);
    spi_send_msg(MAX7219_SHUTDOWN, 0x01UL);
    //spi_send_msg(MAX7219_DISPLAY_TEST, 0x01UL);
    // Clear display
        for (uint8_t i = 1; i <= 8; i++) {
            spi_send_msg(i, 0x00); // Turn off all LEDs in row i
        }
}

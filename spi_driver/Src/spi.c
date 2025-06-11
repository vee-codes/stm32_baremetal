/*
 * spi.c
 *
 *  Created on: Jun 10, 2025
 *      Author: tony
 */
#include "stm32f7xx.h"
// GPIO MODER
#define GPIO_MODER_INPUT 0UL //input mode -reset state
#define GPIO_MODER_OUTPUT 1UL // general purpose output mode
#define GPIO_MODER_ALTERNATE 2UL // alternate function mode
#define GPIO_MODER_ANALOG 3UL // Analog Mode
#define AF5 5UL
void spi_init(SPI_TypeDef *SPI_device){
    //SPI - NSS, SCK, MISO, MOSI
    // SPI1 - PA4,5,6,7 AF5 -b0101
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    // Alternate Function Mode
    GPIOA->MODER &= ~(GPIO_MODER_MODER4|GPIO_MODER_MODER5|GPIO_MODER_MODER6|GPIO_MODER_MODER7);
    GPIOA->MODER |= (GPIO_MODER_ALTERNATE<<GPIO_MODER_MODER4_Pos|
                     GPIO_MODER_ALTERNATE<<GPIO_MODER_MODER5_Pos|
                     GPIO_MODER_ALTERNATE<<GPIO_MODER_MODER6_Pos|
                     GPIO_MODER_ALTERNATE<<GPIO_MODER_MODER7_Pos);
    GPIOA->AFR[0] &= ~(GPIO_AFRL_AFRL4_Msk | GPIO_AFRL_AFRL5_Msk | GPIO_AFRL_AFRL6_Msk | GPIO_AFRL_AFRL7_Msk);
    GPIOA->AFR[0] |= (AF5<<GPIO_AFRL_AFRL4_Pos| AF5<<GPIO_AFRL_AFRL5_Pos |
                      AF5<<GPIO_AFRL_AFRL6_Pos| AF5<<GPIO_AFRL_AFRL7_Pos);
    //SPI 2 and 3 use APB1, all others APB2
    RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;
    // set clock (f/2,4,8...256)
    SPI_device->CR1 |= SPI_CR1_BR_0; // f/4 => 4 MHz
    // set ssm (software controlled CS) - control with SSI bit
    SPI_device->CR1 |= (SPI_CR1_SSM | SPI_CR1_SSI);
    // set data size
    SPI_device->CR2 |= 7UL<<SPI_CR2_DS_Pos;
    // enable SPI peripheral and set as master
    SPI_device->CR1 |= (SPI_CR1_SPE | SPI_CR1_MSTR);
}

void spi_send_msg(uint16_t msg){

}

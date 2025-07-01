#include "usart.h"

void UART3_Init(int peripheral_clock, int baud_rate){
    /*
     * Initializes UART3 (USB) by setting applicaple GPIO to alternate function
     */
    // Enable clocks
    RCC->APB1ENR |= RCC_APB1ENR_USART3EN;  // USART3 clock
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;    // GPIOD clock (PD8=TX, PD9=RX)

    // Configure GPIO (Alternate Function AF7 for USART3)
    GPIOD->MODER &= ~(GPIO_MODER_MODER8_Msk | GPIO_MODER_MODER9_Msk);  // Clear bits
    GPIOD->MODER |= (2UL << GPIO_MODER_MODER8_Pos) | (2UL << GPIO_MODER_MODER9_Pos);  // AF mode
    GPIOD->AFR[1] |= (7UL << (0 * 4)) | (7UL << (1 * 4));      // AF7 (USART3)

    // Configure USART3
    USART3->BRR = (peripheral_clock/ baud_rate);  // 16MHz PCLK1, 115200 baud
    USART3->CR1 = USART_CR1_TE | USART_CR1_UE;  // Enable TX, USART
}

#include <stdio.h>
#include "spi.h"
/*//////////////
 Defines
*///////////////

// Onboard LED pins
#define LD1_PIN 0U //PB0
#define LD2_PIN 7U //PB7
#define LD3_PIN 14U // PB14
// Functions
#define LED_ON(pin) (1U << (pin))
#define LED_OFF(pin) (1U << ((pin) + 16U))
// Systick
#define SYS_CLK 16000000
#define MS_TO_SEC 1000
#define SYS_LOAD_MAX (0x00FFFFFFUL)

//uart
#define BAUD_RATE 115200

/*//////////////
 Declarations
*///////////////

// systick
void delay_ms(int delay);
void systick_config(uint32_t ticks_ms, uint32_t clock_speed);
//spi

// uart
void UART3_Init(void);
void UART3_SendChar(char c);
void UART3_SendString(const char *str);

/*//////////////
 * TYPEDEFS
*///////////////


int main(void){

	// Enable clock access to GPIO B and C
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;

	// set LEDs as outputs
	// Reset value for port B: 0x0000 0280
	GPIOB->MODER |= (1U<<0); //pin0 0-1
	GPIOB->MODER |= (1U<<14); //pin7 14-15
	GPIOB->MODER &=~(1U<<15);
	GPIOB->MODER |= (1U<<28); //pin14 28-29

	UART3_SendString("Starting up...");
	systick_config(1,SYS_CLK); // use 1ms ticks
	UART3_Init();
	spi_init(SPI1);
	delay_ms(1000);
	UART3_SendString("Initialized Peripherals");

	test_leds();
	delay_ms(1000);


	while(1){
		// Turn on the LEDs
	    spi_send_msg(MAX7219_DISPLAY_TEST, 0x01UL);
	    GPIOB->BSRR = LED_ON(LD1_PIN);
		GPIOB->BSRR = LED_ON(LD2_PIN);
		GPIOB->BSRR = LED_ON(LD3_PIN);
		delay_ms(500);
		GPIOB->BSRR = LED_OFF(LD1_PIN);
		GPIOB->BSRR = LED_OFF(LD2_PIN);
		GPIOB->BSRR = LED_OFF(LD3_PIN);
		spi_send_msg(MAX7219_DISPLAY_TEST, 0x00UL);
		delay_ms(600);

	}
}

void delay_ms(int delay){
	for(int i = 0; i < delay; i++){
		while(!(SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk)){
		}
	}
}

void systick_config(uint32_t ticks_ms, uint32_t clock_speed){
	uint32_t load_val = (ticks_ms * clock_speed / MS_TO_SEC)- 1U;

	// clip to max valid value
	if (load_val > SYS_LOAD_MAX){
		load_val = SYS_LOAD_MAX;
	}

	SysTick->LOAD = load_val;
	// Write to current value register clears field to 0
	SysTick->VAL = 0;
	// Enable and set to processor clock
	SysTick->CTRL |= (SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk);

}

void UART3_Init(void){
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
    USART3->BRR = (SYS_CLK/ BAUD_RATE);  // 16MHz PCLK1, 115200 baud
    USART3->CR1 = USART_CR1_TE | USART_CR1_UE;  // Enable TX, USART
}

void UART3_SendChar(char c){
	// Wait for TX buffer empty
	while(!(USART3->ISR & USART_ISR_TXE));
	USART3->TDR = c;
}

void UART3_SendString(const char *str){
    while(*str) UART3_SendChar(*str++);
}

#include "stm32f7xx.h"

// Onboard LED pins
#define LD1_PIN 0U //PB0
#define LD2_PIN 7U //PB7
#define LD3_PIN 14U // PB14

#define GPIOBEN (1U<<1)
#define GPIOCEN (1U<<2)

#define LED_ON(pin) (1U << (pin))
#define LED_OFF(pin) (1U << ((pin) + 16U))
#define USR_BTN (1U<<13) // PC13

void delay(uint32_t milliseconds) {
	for (uint32_t i = 0; i < milliseconds * 1000; i++) {
		__NOP();
	}
}

int main(void) {
	// Enable clock access to GPIO B and C
	RCC->AHB1ENR |= GPIOBEN;
    RCC->AHB1ENR |= GPIOCEN;

	// set LEDs as outputs
	// Reset value for port B: 0x0000 0280
	GPIOB->MODER |= (1U<<0); //pin0
	GPIOB->MODER |= (1U<<14); //pin7
	GPIOB->MODER &=~(1U<<15);
	GPIOB->MODER |= (1U<<28); //pin14
    // Set btn as input
    GPIOC->MODER &=~(1U<<26);
    GPIOC->MODER &=~(1U<<27);
    //GPIOC->PUPDR |=(1U<<27);

	while(1){
		// Turn on the LEDs
        if (!(GPIOC->IDR & USR_BTN)) {
        	GPIOB->BSRR = LED_ON(LD1_PIN);
			delay(500);
			GPIOB->BSRR = LED_ON(LD2_PIN);
			delay(500);
			GPIOB->BSRR = LED_ON(LD3_PIN);
			delay(500);
        }
        else {
		// Turn off the LEDs
		GPIOB->BSRR = LED_OFF(LD1_PIN);
		GPIOB->BSRR = LED_OFF(LD2_PIN);
		GPIOB->BSRR = LED_OFF(LD3_PIN);
        }
        delay(5);
	}
}

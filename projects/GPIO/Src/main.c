#include "stm32f7xx.h"

#define GPIOBEN (1U<<1)
#define LD1 (1U<<0)
#define LD2 (1U<<7)
#define LD3 (1U<<14)

int main(void) {
	// Enable clock access to GPIOB
	RCC->AHB1ENR |= GPIOBEN;
	// set LEDs as outputs
	// Reset value for port B: 0x0000 0280
	GPIOB->MODER |= (1U<<0); //pin0
	GPIOB->MODER |= (1U<<14); //pin7
	GPIOB->MODER &=~(1U<<15);
	GPIOB->MODER |= (1U<<28); //pin14
	while(1){
		GPIOB->ODR ^= LD1;
		for (int i = 0;i<1000000;i++){}
		GPIOB->ODR ^= LD2;
		for (int i = 0;i<1000000;i++){}
		GPIOB->ODR ^= LD3;
		for (int i = 0;i<1000000;i++){}
	}
}

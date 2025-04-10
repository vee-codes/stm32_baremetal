// On-board LEDs

// LD1,2,3
// green PB0 (SB120 ON and SB119 OFF) or PA5 (SB119 ON and SB120 OFF)
// blue PB7
// red PB14
// PB maps to GPIOB


#define PERIPH_BASE (0x40000000UL)
#define AHB1_OFFSET (0x00020000UL)
#define AHB1_BASE (PERIPH_BASE + AHB1_OFFSET)

//GPIOA
#define GPIOA_BASE AHB1_BASE

// GPIOB
#define GPIOB_OFFSET (0x0400UL)
#define GPIOB_BASE (GPIOB_OFFSET + AHB1_BASE)

//RCC
#define RCC_OFFSET (0x3800UL)
#define RCC_BASE (AHB1_BASE + RCC_OFFSET)
// RCC_AHB1ENR - 32 bits
#define RCC_AHB1ENR_OFFSET (0x30UL)
#define RCC_AHB1ENR (*(volatile unsigned int *)(RCC_BASE + RCC_AHB1ENR_OFFSET))
#define GPIOBEN (1U<<1)

// GPIOx_MODER
#define GPIOx_MODER_OFFSET (0x00UL) // x = A to K
#define GPIOB_MODER_BASE (*(volatile unsigned int *)(GPIOB_BASE + GPIOx_MODER_OFFSET))

// GPIOx_ODR - 32 bits | x = A to K
#define GPIOx_ODR_OFFSET (0x14UL)
#define GPIOx_ODR_BASE (*(volatile unsigned int *)(GPIOB_BASE + GPIOx_ODR_OFFSET))

#define PIN0 (1U<<0)
#define PIN7 (1U<<7)
#define PIN14 (1U<<14)
#define GREEN PIN0
#define BLUE PIN7
#define RED PIN14


int main(void) {
	// Enable clock access to GPIOB
	RCC_AHB1ENR |= GPIOBEN;
	// set LEDs as outputs
	// Reset value for port B: 0x0000 0280
	GPIOB_MODER_BASE |= (1U<<0); //pin0
	GPIOB_MODER_BASE |= (1U<<14); //pin7
	GPIOB_MODER_BASE &=~(1U<<15);
	GPIOB_MODER_BASE |= (1U<<28); //pin14
	while(1){
		GPIOx_ODR_BASE ^= GREEN;
		for (int i = 0;i<1000000;i++){}
		GPIOx_ODR_BASE ^= BLUE;
		for (int i = 0;i<1000000;i++){}
		GPIOx_ODR_BASE ^= RED;
		for (int i = 0;i<1000000;i++){}
	}
}

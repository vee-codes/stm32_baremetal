// On-board LEDs

// LD1,2,3
// green PB0 (SB120 ON and SB119 OFF) or PA5 (SB119 ON and SB120 OFF)
// blue PB7
// red PB14
// PB maps to GPIOB

#include<stdint.h>

//RCC Struct
typedef struct {
    volatile uint32_t CR;
    volatile uint32_t PLLCFGR;
    volatile uint32_t CFGR;
    volatile uint32_t CIR;
    volatile uint32_t AHB1RSTR;
    volatile uint32_t AHB2RSTR;
    volatile uint32_t AHB3RSTR;
    volatile uint32_t RESERVED0;
	volatile uint32_t APB1RSTR;
    volatile uint32_t APB2RSTR;
    volatile uint32_t RESERVED1;
    volatile uint32_t RESERVED2;
	volatile uint32_t AHB1ENR;
	volatile uint32_t AHB2ENR;
	volatile uint32_t AHB3ENR;
    volatile uint32_t RESERVED3;
    volatile uint32_t APB1ENR;
    volatile uint32_t APB2ENR;
    volatile uint32_t RESERVED4;
    volatile uint32_t RESERVED5;
    volatile uint32_t AHB1LPENR;
    volatile uint32_t AHB2LPENR;
    volatile uint32_t AHB3LPENR;
    volatile uint32_t RESERVED6;
    volatile uint32_t APB1LPENR;
    volatile uint32_t APB2LPENR;
    volatile uint32_t RESERVED7;
    volatile uint32_t RESERVED8;
    volatile uint32_t BDCR;
    volatile uint32_t CSR;
    volatile uint32_t RESERVED9;
    volatile uint32_t RESERVED10;
    volatile uint32_t SSCGR;
    volatile uint32_t PLLI2SCFGR;
    volatile uint32_t RPLLI2SAICFGR;
    volatile uint32_t DCKCFGR1;
    volatile uint32_t DCKCFGR2;
} RCCReg_t;

// GPIO Register Map
typedef struct {
	volatile uint32_t MODER;
	volatile uint32_t OTYPER;
	volatile uint32_t OSPEEDR;
	volatile uint32_t PUPDR;
	volatile uint32_t IDR;
	volatile uint32_t ODR;
	volatile uint32_t BSRR;
	volatile uint32_t LCKR;
	volatile uint32_t AFRL;
	volatile uint32_t AFRH;
} GPIOxReg_t;

#define PERIPH_BASE (0x40000000UL)
#define AHB1_OFFSET (0x00020000UL)
#define AHB1_BASE (PERIPH_BASE + AHB1_OFFSET)

//GPIO
#define GPIOB_OFFSET (0x0400UL)
#define GPIOB_BASE (GPIOB_OFFSET + AHB1_BASE)
#define GPIOB ((GPIOxReg_t*) GPIOB_BASE)

// RCC
#define RCC_OFFSET (0x3800UL)
#define RCC_BASE (AHB1_BASE + RCC_OFFSET)
#define RCC ((RCCReg_t*) RCC_BASE)

#define LD1 (1U<<0)
#define LD2 (1U<<7)
#define LD3 (1U<<14)
#define GPIOBEN (1U<<1) // Enabled GPIOB

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

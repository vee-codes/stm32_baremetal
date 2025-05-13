#include "stm32f7xx.h"
#include <stdio.h>
// Defines
#define MPU_6050_ADDR 0x68UL
#define ACCEL_REG 0x3B
#define LED_ON(pin) (1U << (pin))
#define LED_OFF(pin) (1U << ((pin) + 16U))
// Onboard LED pins
#define LD1_PIN 0U //PB0
#define LD2_PIN 7U //PB7
#define LD3_PIN 14U // PB14

#define SYS_CLK 16000000
#define MS_TO_SEC 1000
#define SYS_LOAD_MAX (0x00FFFFFFUL)


//Declarations
void delay_ms(int delay);
void systick_config(uint32_t ticks_ms, uint32_t clock_speed);
void i2c1_init(void);
void i2c1_read_byte(char saddr, char reg_addr, char *data);
//
void UART3_Init(void);
void UART3_SendChar(char c);
void UART3_SendString(const char *str);

int main(void) {

	// Enable clock access to GPIO B and C
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;

	// set LEDs as outputs
	// Reset value for port B: 0x0000 0280
	GPIOB->MODER |= (1U<<0); //pin0 0-1
	GPIOB->MODER |= (1U<<14); //pin7 14-15
	GPIOB->MODER &=~(1U<<15);
	GPIOB->MODER |= (1U<<28); //pin14 28-29
	systick_config(1,SYS_CLK); // use 1ms ticks
	UART3_Init();
	while(1){
		// Turn on the LEDs
		GPIOB->BSRR = LED_ON(LD1_PIN);
		GPIOB->BSRR = LED_ON(LD2_PIN);
		GPIOB->BSRR = LED_ON(LD3_PIN);
		delay_ms(1000);

		GPIOB->BSRR = LED_OFF(LD1_PIN);
		GPIOB->BSRR = LED_OFF(LD2_PIN);
		GPIOB->BSRR = LED_OFF(LD3_PIN);
		delay_ms(1000);

		i2c1_init();
		char measurement;
		i2c1_read_byte(MPU_6050_ADDR, ACCEL_REG, &measurement);
		printf("%d",measurement);


		UART3_SendString("Hello, UART!\r\n");

	}
}

void delay_ms(int delay){
	for(int i = 0; i < delay; i++) {
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

void i2c1_init(void){
	// GPIO
	// Enable clock GPIOB
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
	// Set Pins 6 and 7 to AF (b10)
	GPIOB->MODER &= ~(GPIO_MODER_MODER6_Msk) | (GPIO_MODER_MODER7_Msk);
	GPIOB->MODER &= (GPIO_MODER_MODER6_Msk | GPIO_MODER_MODER7_Msk);
	GPIOB->MODER |= ((0x2UL<<GPIO_MODER_MODER6_Pos)| (0x2UL<<GPIO_MODER_MODER7_Pos));
	// I2C requires Open-Drain connections and a pullup
	GPIOB->OTYPER |= (GPIO_OTYPER_OT6 | GPIO_OTYPER_OT7);
	GPIOB->PUPDR &= ~(GPIO_PUPDR_PUPDR6_Msk| GPIO_PUPDR_PUPDR7_Msk);
	GPIOB->PUPDR |= (1UL<<GPIO_PUPDR_PUPDR6_Pos | (1UL<<GPIO_PUPDR_PUPDR7_Pos));
	// I2C
	// Enable clock APB1
	RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;
	// Alternate Function I2c-> AF4
	GPIOB->AFR[0] |= GPIO_AFRL_AFRL4;
	// I2C1_SCL-PB6, I2C1_SDA-PB7
	I2C1->CR1 &= ~(I2C_CR1_PE);
	// Set Clocks (16 Mhz reference)
	I2C1->TIMINGR |= (0x3UL<<I2C_TIMINGR_PRESC_Pos \
					  | 0x13UL<<I2C_TIMINGR_SCLL_Pos \
					  | 0xFUL<<I2C_TIMINGR_SCLH_Pos \
					  | 0x2UL<<I2C_TIMINGR_SDADEL_Pos \
					  | 0x4<<I2C_TIMINGR_SCLDEL_Pos);
	I2C1->CR1 &= ~(I2C_CR1_NOSTRETCH);

	I2C1->CR1 |= I2C_CR1_PE;

}


void i2c1_read_byte(char saddr, char reg_addr, char *data){
	// wait until busy flag is clear
	while(I2C_ISR_BUSY){}
	I2C1->CR2 |= I2C_CR2_START;

	I2C1->CR2 |= (saddr<<1);
	// 0 for writes, 1 for reads
	I2C1->CR2 &= ~I2C_CR2_RD_WRN;
	// num bytes to send
	I2C1->CR2 |= (1UL<<I2C_CR2_NBYTES_Pos);
	// Send the register of interest
	I2C1->TXDR = reg_addr;
	// wait until received data
	while(I2C_ISR_RXNE){}
	//
	*data++ = I2C1->RXDR;

	I2C2->CR2 |= (1UL<<I2C_CR2_STOP_Pos);
}

void UART3_Init(void) {
    // 1. Enable clocks
    RCC->APB1ENR |= RCC_APB1ENR_USART3EN;  // USART3 clock
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;    // GPIOD clock (PD8=TX, PD9=RX)

    // 2. Configure GPIO (Alternate Function AF7 for USART3)
    GPIOD->MODER &= ~(GPIO_MODER_MODER8 | GPIO_MODER_MODER9);  // Clear bits
    GPIOD->MODER |= (2UL << GPIO_MODER_MODER8_Pos) | (2UL << GPIO_MODER_MODER9_Pos);  // AF mode
    GPIOD->AFR[1] |= (7UL << (0 * 4)) | (7UL << (1 * 4));      // AF7 (USART3)

    // 3. Configure USART3
    USART3->BRR = (16000000 / 115200);  // 16MHz PCLK1, 115200 baud
    USART3->CR1 = USART_CR1_TE | USART_CR1_UE;  // Enable TX, USART
}

void UART3_SendChar(char c) {
    while (!(USART3->ISR & USART_ISR_TXE));  // Wait for TX buffer empty
    USART3->TDR = c;
}

void UART3_SendString(const char *str) {
    while (*str) UART3_SendChar(*str++);
}


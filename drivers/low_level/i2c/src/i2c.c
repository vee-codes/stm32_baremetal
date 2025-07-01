#include "i2c.h"


void i2c1_init(){
	// GPIO
	// Enable clock GPIOB
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
	// Set Pins 8 and 9 to AF (b10)
	GPIOB->MODER &= ~((GPIO_MODER_MODER8_Msk) | (GPIO_MODER_MODER9_Msk));
	GPIOB->MODER |= ((0x2UL<<GPIO_MODER_MODER8_Pos)| (0x2UL<<GPIO_MODER_MODER9_Pos));
	// I2C requires Open-Drain connections
	GPIOB->OTYPER |= (GPIO_OTYPER_OT8 | GPIO_OTYPER_OT9);
	GPIOB->PUPDR &= ~(GPIO_PUPDR_PUPDR8_Msk| GPIO_PUPDR_PUPDR9_Msk);
	// Uncomment below if external pull-up resistors are not implemented
	//GPIOB->PUPDR |= (1UL<<GPIO_PUPDR_PUPDR8_Pos | (1UL<<GPIO_PUPDR_PUPDR9_Pos));
	// I2C
	// Enable clock APB1
	RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;
	// Alternate Function I2c-> AF4
	GPIOB->AFR[1] &= ~(GPIO_AFRH_AFRH0_Msk| GPIO_AFRH_AFRH1_Msk);
	GPIOB->AFR[1] |= ((0x4UL<<GPIO_AFRH_AFRH0_Pos)| (0x4UL<<GPIO_AFRH_AFRH1_Pos));
	// I2C1_SCL-PB6, I2C1_SDA-PB7
	I2C1->CR1 &= ~(I2C_CR1_PE);
	// Per the ref man. PE must be kept low during at least three APB clock cycles to perform the I2C reset.
	// The manual recommends setting PE=0, checking PE=0, then setting PE=1
	// It was observed that invalid i2c data was obtained when trying to read right after init without this sequence.
	while(I2C1->CR1 & I2C_CR1_PE);
	// Set Clocks (16 Mhz reference)
	// FAST MODE (400kHz)
	I2C1->TIMINGR |=   (0x1UL<<I2C_TIMINGR_PRESC_Pos
					  | 0x9UL<<I2C_TIMINGR_SCLL_Pos
					  | 0x3UL<<I2C_TIMINGR_SCLH_Pos
					  | 0x2UL<<I2C_TIMINGR_SDADEL_Pos
					  | 0x3UL<<I2C_TIMINGR_SCLDEL_Pos);

	I2C1->CR1 &= ~(I2C_CR1_NOSTRETCH);
	I2C1->CR1 |= I2C_CR1_PE;
}

void i2c1_read_byte(uint8_t saddr, uint8_t reg_addr, uint8_t *data, size_t num_bytes){
	/*
	 * Reads num_bytes of data from a i2c device register
	 * writes that data to the pointer that is passed
	 */
	// wait until busy flag is clear
	while(I2C1->ISR & I2C_ISR_BUSY);

	// 0 for writes, 1 for reads
	I2C1->CR2 = (saddr<<1);
	I2C1->CR2 &= ~I2C_CR2_ADD10;
	I2C1->CR2 |= (1U)<<I2C_CR2_NBYTES_Pos;
	// Send the register of interest
	while(!(I2C1->ISR & I2C_ISR_TXE));
	I2C1->TXDR = reg_addr;
	I2C1->CR2 |= I2C_CR2_START;
	// Wait until transfer is complete
	while(!(I2C1->ISR & I2C_ISR_TC));
	while(I2C1->CR2 & I2C_CR2_START);
	// Set to read mode
	I2C1->CR2 = (saddr << 1) | (num_bytes << I2C_CR2_NBYTES_Pos) | (1 << I2C_CR2_RD_WRN_Pos);
	I2C1->CR2 |= I2C_CR2_START;

	for (int i = 0; i < num_bytes; i++){
	    while(!(I2C1->ISR & I2C_ISR_RXNE));  // Wait for RX buffer
	    data[i] = I2C1->RXDR;  // Store received byte
	}
	// set stop after transfer complete then clear the flag
	while(!(I2C1->ISR & I2C_ISR_TC));
	I2C1->CR2 |= I2C_CR2_STOP;
	while(!(I2C1->ISR & I2C_ISR_STOPF));
	(void)I2C1->ISR;
	I2C1->ICR |= I2C_ICR_STOPCF;
}

void i2c1_write_byte(uint8_t saddr, uint8_t reg_addr, uint8_t data, size_t num_bytes){
	/*
	 * Writes to a register of the i2c device
	 */
	while(I2C1->ISR & I2C_ISR_BUSY);
	// 0 for writes, 1 for reads
	I2C1->CR2 = (saddr<<1);
	I2C1->CR2 &= ~I2C_CR2_ADD10;
	I2C1->CR2 |= (2U)<<I2C_CR2_NBYTES_Pos;
	// Send the register of interest
	while(!(I2C1->ISR & I2C_ISR_TXE));
	I2C1->TXDR = reg_addr;
	I2C1->CR2 |= I2C_CR2_START;
	// Send data when the register is empty
	while(!(I2C1->ISR & I2C_ISR_TXE));
	I2C1->TXDR = data;
	// Set stop after transfer complete then clear the flag
	while(!(I2C1->ISR & I2C_ISR_TC));
	I2C1->CR2 |= I2C_CR2_STOP;
	while(!(I2C1->ISR & I2C_ISR_STOPF));
	I2C1->CR2 |= I2C_ICR_STOPCF | I2C_ICR_ADDRCF;
}

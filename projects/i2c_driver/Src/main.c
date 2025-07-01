#include "stm32f7xx.h"
#include "i2c.h"
/*//////////////
 Defines
*///////////////

// Device - MPU6050
#define MPU_6050_ADDR 0x68 // Slave Address
#define ACCEL_REG 0x3B // Register 59 - Accel_XOUT_H
#define PWR_MGMT_1 0x6B //Register 104 - Power Management 1
#define WHO_AM_I 0x75
#define TEMP_H 0x41
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

#define I2C_TIMING_100HZ 0x00303D5B //https://blog.embeddedexpert.io/?p=902
//uart
#define BAUD_RATE 115200

/*//////////////
 Declarations
*///////////////

// systick
void delay_ms(int delay);
void systick_config(uint32_t ticks_ms, uint32_t clock_speed);
// i2c
void i2c1_init(void);
void i2c1_read_byte(uint8_t saddr, uint8_t reg_addr, uint8_t *data, size_t num_bytes);
void i2c1_write_byte(uint8_t saddr, uint8_t reg_addr, uint8_t data, size_t num_bytes);
uint8_t I2C_ReadByte(I2C_TypeDef *I2Cx, uint8_t slaveAddr, uint8_t regAddr);

// uart
void UART3_Init(void);
void UART3_SendChar(char c);
void UART3_SendString(const char *str);
// mpu
void mpu_init();
void MPU6050_ReadAccel(int16_t *accel);
void MPU_6050_TEMP_IN_C(int16_t *temp);

/*//////////////
 * TYPEDEFS
*///////////////

typedef struct {
	int16_t accel_x;
	int16_t accel_y;
	int16_t accel_z;
} ACCEL_3D;

typedef struct {
	int16_t gyro_x;
	int16_t gyro_y;
	int16_t gyro_z;
} GYRO_3D;

ACCEL_3D *mpu_accel;
GYRO_3D *mpu_gyro;

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
	i2c1_init();
	delay_ms(1000);
	uint8_t dev_addr;
	i2c1_read_byte(MPU_6050_ADDR, WHO_AM_I, &dev_addr, 1);
	mpu_init();
	UART3_SendString("Initialized Peripherals");
	int16_t accel_raw[3];  // X, Y, Z
	int16_t temp_raw[1];

	while(1){
		// Turn on the LEDs
	    GPIOB->BSRR = LED_ON(LD1_PIN);
		GPIOB->BSRR = LED_ON(LD2_PIN);
		GPIOB->BSRR = LED_ON(LD3_PIN);
		MPU6050_ReadAccel(accel_raw);
		MPU_6050_TEMP_IN_C(temp_raw);
		delay_ms(500);
		GPIOB->BSRR = LED_OFF(LD1_PIN);
		GPIOB->BSRR = LED_OFF(LD2_PIN);
		GPIOB->BSRR = LED_OFF(LD3_PIN);
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

void mpu_init(){
	// reset the mpu and wake it up
	uint8_t init_data = 0;
	i2c1_write_byte(MPU_6050_ADDR, PWR_MGMT_1, init_data, 2);
	delay_ms(1000);
}

void MPU6050_ReadAccel(int16_t *accel){
	size_t num_registers = 6; // 6 registers of 1 byte each
    uint8_t data[num_registers]; // each axis has an upper and lower byte

    i2c1_read_byte(MPU_6050_ADDR, ACCEL_REG, data, num_registers);

    // Combine high/low bytes and convert to int16_t
    accel[0] = (data[0] << 8) | data[1];  // X-axis
    accel[1] = (data[2] << 8) | data[3];  // Y-axis
    accel[2] = (data[4] << 8) | data[5];  // Z-axis

}

void MPU_6050_TEMP_IN_C(int16_t *temp){
	size_t num_registers = 2; // 2 registers: upper and lower byte
	uint8_t data[num_registers];

	i2c1_read_byte(MPU_6050_ADDR, TEMP_H, data, num_registers);

}


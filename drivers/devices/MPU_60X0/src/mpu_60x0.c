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
#ifndef MPU_60X0_H_
#define MPU_60X0_H_

// MPU 6050 is i2c only, MPU 6000 is spi or i2c
#include "../low_level/spi/spi.h"
#include "../low_level/spi/i2c.h"


void mpu_init();
void MPU6050_ReadAccel(int16_t *accel);
void MPU_6050_TEMP_IN_C(int16_t *temp);

#endif
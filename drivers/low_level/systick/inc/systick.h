#ifndef SYSTICK_H_
#define SYSTICK_H_
#include "stm32f767xx.h"
void systick_config(uint32_t ticks_ms, uint32_t clock_speed);

#endif
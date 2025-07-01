#ifndef SYSTICK_H
#define SYSTICK_H
#include "stm32f7xx.h"

/*
 * DEFINES
 */
#define SYS_CLK 16000000
#define MS_TO_SEC 1000
#define SYS_LOAD_MAX (0x00FFFFFFUL)

/*
 * DECLARATIONS
 */
void systick_init(uint32_t ticks_ms, uint32_t clock_speed);

#endif

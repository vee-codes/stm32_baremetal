#ifndef SERIAL_LOGGER_H_
#define SERIAL_LOGGER_H_
#include "../low_level/usart/usart.h"
void UART3_SendChar(char c);
void UART3_SendString(const char *str);
#endif
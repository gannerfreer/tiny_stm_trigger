#ifndef SERIAL_LOG_H
#define SERIAL_LOG_H

#include "stm32f0xx_hal.h"

void SerialLog_Init(UART_HandleTypeDef *huart);
void SerialLog_Print(const char *text);
void SerialLog_Printf(const char *fmt, ...);

#endif /* SERIAL_LOG_H */

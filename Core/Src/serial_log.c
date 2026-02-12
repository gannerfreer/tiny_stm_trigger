#include "serial_log.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#define SERIAL_LOG_BUFFER_SIZE 192U
#define SERIAL_LOG_TX_TIMEOUT_MS 20U

static UART_HandleTypeDef *serial_log_uart;

void SerialLog_Init(UART_HandleTypeDef *huart)
{
  serial_log_uart = huart;
}

void SerialLog_Print(const char *text)
{
  size_t len;

  if ((serial_log_uart == NULL) || (text == NULL))
  {
    return;
  }

  len = strlen(text);
  if (len == 0U)
  {
    return;
  }

  (void)HAL_UART_Transmit(serial_log_uart, (uint8_t *)text, (uint16_t)len, SERIAL_LOG_TX_TIMEOUT_MS);
}

void SerialLog_Printf(const char *fmt, ...)
{
  va_list args;
  int length;
  char buffer[SERIAL_LOG_BUFFER_SIZE];

  if ((serial_log_uart == NULL) || (fmt == NULL))
  {
    return;
  }

  va_start(args, fmt);
  length = vsnprintf(buffer, sizeof(buffer), fmt, args);
  va_end(args);

  if (length <= 0)
  {
    return;
  }

  if (length > (int)(sizeof(buffer) - 1U))
  {
    length = (int)(sizeof(buffer) - 1U);
  }

  (void)HAL_UART_Transmit(serial_log_uart, (uint8_t *)buffer, (uint16_t)length, SERIAL_LOG_TX_TIMEOUT_MS);
}

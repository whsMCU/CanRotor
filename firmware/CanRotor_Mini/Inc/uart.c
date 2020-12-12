/*
 * uart.c
 *
 *  Created on: 2020. 12. 12.
 *      Author: WANG
 */
#include "Board.h"
#include "uart.h"
#include "Queue.h"

static bool uart_ch[UART_MAX_CH];

bool uartInit(void)
{
  int i = 0;
  for (i=0; i<UART_MAX_CH; i++)
  {
    uart_ch[i] = false;
  }
  return true;
}

bool uartOpen(uint8_t ch, uint32_t baud)
{
  bool ret = false;
  switch(ch)
  {
    case _DEF_UART1:
      uart_ch[ch] = true;
      ret = true;
      break;
  }
  return ret;

}

uint32_t uartAvailable(uint8_t ch)
{
  uint32_t ret = 0;

  switch(ch)
  {
    case _DEF_UART1:
      ret = QueueAvailable(&Q_buffer[ch]);
      break;
  }

  return ret;

}

uint8_t  uartRead(uint8_t ch)
{
  uint8_t ret = 0;

  switch(ch)
  {
    case _DEF_UART1:
      ret = read_Q(&Q_buffer[ch]);
      break;
  }

  return ret;
}

HAL_StatusTypeDef uartWrite(uint8_t ch, uint8_t *p_data, uint16_t lenght)
{
  HAL_StatusTypeDef ret = 0;

  switch(ch)
  {
    case _DEF_UART1:
      ret = HAL_UART_Transmit_DMA(&huart1, (uint8_t *)p_data, lenght);
      break;
    case _DEF_UART2:
      ret = HAL_UART_Transmit_DMA(&huart2, (uint8_t *)p_data, lenght);
      break;
  }

  return ret;

}

uint32_t uartPrintf(uint8_t ch, char *fmt, ...)
{
  char buf[256];
  va_list args;
  int len;
  HAL_StatusTypeDef ret;

  va_start(args, fmt);
  len = vsnprintf(buf, 256, fmt, args);
  ret = uartWrite(ch, (uint8_t *)buf, len);

  va_end(args);

  return ret;
}

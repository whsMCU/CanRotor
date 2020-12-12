/*
 * uart.h
 *
 *  Created on: 2020. 12. 12.
 *      Author: WANG
 */

#ifndef UART_H_
#define UART_H_

#include "hw_def.h"

#ifdef _USE_HW_UART

bool     uartInit(void);
bool     uartOpen(uint8_t ch, uint32_t baud);
uint32_t uartAvailable(uint8_t ch);
uint8_t  uartRead(uint8_t ch);
HAL_StatusTypeDef uartWrite(uint8_t ch, uint8_t *p_data, uint16_t lenght);
uint32_t uartPrintf(uint8_t ch, char *fmt, ...);

#endif

#endif /* UART_H_ */

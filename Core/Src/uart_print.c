/*
 * uart_print.c
 *
 *  Created on: Nov 25, 2022
 *      Author: ADMIN
 */
#include "uart_print.h"

void USART_Init(USART_H_t *USART_handler, UART_HandleTypeDef *huart)
{
	USART_handler->_huart = huart;
}

void UART_sendByte(USART_H_t *USART_handler, uint8_t data)
{
    HAL_UART_Transmit(USART_handler->_huart, &data, 1, 100);
}

void USART_sendChar(USART_H_t *USART_handler, uint8_t data)
{
    UART_sendByte(USART_handler, data);
}

void USART_put(USART_H_t *USART_handler, char *data)
{
	while (data[0] != '\0')
	{
		USART_sendChar(USART_handler, data[0]);
		data++;
	}
}

int USART_Transmit(USART_H_t *USART_handler, const char *fmt, ...)
{
	int done;
	va_list args;
	static char buffer[256];
	va_start(args, fmt);

	done = vsnprintf(buffer, 256, fmt, args);

	USART_put(USART_handler, buffer);

	va_end(args);
	return done;
}


/*
 * uart_print.h
 *
 *  Created on: Nov 25, 2022
 *      Author: ADMIN
 */

#ifndef INC_UART_PRINT_H_
#define INC_UART_PRINT_H_

#include <stdarg.h>
#include "main.h"

typedef struct{
	UART_HandleTypeDef *_huart;
}USART_H_t;

void USART_Init(USART_H_t *USART_handler, UART_HandleTypeDef *huart);
int USART_Transmit(USART_H_t *USART_handler, const char *fmt, ...);

#endif /* INC_UART_PRINT_H_ */

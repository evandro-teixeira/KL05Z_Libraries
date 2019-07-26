/*
 * uart.h
 *
 *  Created on: 23/07/2019
 *      Author: Evandro Teixeira
 */

#ifndef KL05Z_LIBRARIES_UART_H_
#define KL05Z_LIBRARIES_UART_H_

#include "kl05_libraries.h"

typedef enum
{
	Option_Pins_0 = 0,
	Option_Pins_1,
}uart_pins_t;

bool uart_init (UART0_MemMapPtr uartch,uart_pins_t pins, uint32_t sysclk, uint32_t baud);
char uart_getchar (UART0_MemMapPtr channel);
void uart_putchar (UART0_MemMapPtr channel, char ch);
void uart_put_string(UART0_MemMapPtr channel, char *txt);
bool uart_enable_irq(UART0_MemMapPtr channel);
bool uart_set_callback_irq(UART0_MemMapPtr channel,void (*task)(void));

#endif /* KL05Z_LIBRARIES_UART_H_ */

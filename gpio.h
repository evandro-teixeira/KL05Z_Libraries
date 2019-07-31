/*
 * gpio.h
 *
 *  Created on: 22/07/2019
 *      Author: Evandro Teixeira
 */

#ifndef KL05Z_LIBRARIES_GPIO_H_
#define KL05Z_LIBRARIES_GPIO_H_

#include "kl05_libraries.h"

typedef enum
{
	GPIO_OUTPUT = 0,
	GPIO_INPUT,
}gpio_dir_config_t;

typedef enum
{
	GPIO_PULLDOWN_RESISTOR,
	GPIO_PULLUP_RESISTOR
}gpio_resistor_t;

typedef enum
{
	GPIO_DISABLE = 0,
	GPIO_ENABLE
}gpio_status_t;


typedef enum
{
	DMA_Request_Disabled = 0, 	// 0000 Interrupt/DMA request disabled.
	DMA_Request_on_Rising_Edge,	// 0001 DMA request on rising edge.
	DMA_Request_on_Falling_Edge,// 0010 DMA request on falling edge.
	DMA_Request_on_Either_Edge,	// 0011 DMA request on either edge.
	Interrupt_when_Logic_Zero,	// 1000 Interrupt when logic zero.
	Interrupt_on_Rising_Edge,	// 1001 Interrupt on rising edge.
	Interrupt_on_Falling_Edge,	// 1010 Interrupt on falling edge.
	Interrupt_on_Either_Edge,	// 1011 Interrupt on either edge.
	Interrupt_When_Logic_One,	// 1100 Interrupt when logic one
}gpio_interrupt_t;

bool gpio_init (GPIO_MemMapPtr gpio,uint32_t pin,gpio_dir_config_t type);
bool gpio_resistor_enable(GPIO_MemMapPtr gpio,uint32_t pin,gpio_resistor_t resistor);
gpio_status_t gpio_read(GPIO_MemMapPtr gpio,uint32_t pin);
void gpio_write(GPIO_MemMapPtr gpio,uint32_t pin,gpio_status_t value);
void gpio_toggle(GPIO_MemMapPtr gpio,uint32_t pin);
bool gpio_interrupt_configuration(GPIO_MemMapPtr gpio,uint32_t pin,gpio_interrupt_t interrupt);
bool gpio_set_callback_irq(GPIO_MemMapPtr gpio, void (*task)(void));
bool gpio_clear_flag(GPIO_MemMapPtr gpio, uint32_t pin);

#endif /* KL05Z_LIBRARIES_GPIO_H_ */

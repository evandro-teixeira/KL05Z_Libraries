/*
 * gpio.c
 *
 *  Created on: 22/07/2019
 *      Author: Evandro Teixeira
 */

#include "gpio.h"

/**
 * @brief
 */
void (*gpio_porta_irq)(void);
void (*gpio_portb_irq)(void);

/**
 * @brief
 * @param gpio
 * @param pin
 * @param type
 * @return
 */
bool gpio_init (GPIO_MemMapPtr gpio,uint32_t pin,gpio_dir_config_t type)
{
	if(gpio == GPIOA)
	{
		// System Clock Gating
		SIM_SCGC5 |= SIM_SCGC5_PORTA_MASK;
		// Pin Mux Control ~ GPIO
		PORT_PCR_REG(PORTA_BASE_PTR,pin) = PORT_PCR_MUX(1);
	}
	else if(gpio == GPIOB)
	{
		// System Clock Gating
		SIM_SCGC5 |= SIM_SCGC5_PORTB_MASK;
		// Pin Mux Control ~ GPIO
		PORT_PCR_REG(PORTB_BASE_PTR,pin) = PORT_PCR_MUX(1);
	}
	else
	{
		return false;
	}

	switch(type)
	{
		case GPIO_OUTPUT:
			gpio->PDDR |= (1 << pin);
		break;
		case GPIO_INPUT:
		default:
			gpio->PDDR &=~(1 << pin);
		break;
	}

	return true;
}

/**
 * @brief
 * @param gpio
 * @param pin
 * @param resistor
 * @return
 */
bool gpio_resistor_enable(GPIO_MemMapPtr gpio,uint32_t pin,gpio_resistor_t resistor)
{
	PORT_MemMapPtr port;

	if(pin >= 32)
	{
		return false;
	}

	if(gpio == GPIOA)
	{
		port = PORTA;
	}
	else if(gpio == GPIOB)
	{
		port = PORTB;
	}
	else
	{
		return false;
	}

	PORT_PCR_REG(port,pin) |= PORT_PCR_PE(1);
	if(resistor == GPIO_PULLDOWN_RESISTOR)
	{
		PORT_PCR_REG(port,pin) |= PORT_PCR_PS(0);
	}
	else
	{
		PORT_PCR_REG(port,pin) |= PORT_PCR_PS(1);
	}

	return true;
}

/**
 * @brief
 * @param gpio
 * @param pin
 * @return
 */
gpio_status_t gpio_read(GPIO_MemMapPtr gpio,uint32_t pin)
{
	if( gpio->PDIR & (1 << pin) )
	{
		return GPIO_ENABLE;
	}
	else
	{
		return GPIO_DISABLE;
	}
}

/**
 * @brief gpio
 * @param pin
 * @param value
 */
void gpio_write(GPIO_MemMapPtr gpio,uint32_t pin,gpio_status_t value)
{
	if(value == GPIO_ENABLE)
	{
		gpio->PSOR |= (1 << pin);
	}
	else
	{
		gpio->PDOR &=~ (1 << pin);
	}
}

/**
 * @brief
 * @param gpio
 * @param pin
 */
void gpio_toggle(GPIO_MemMapPtr gpio,uint32_t pin)
{
	gpio->PTOR = (1 << pin);
}

/**
 * @brief
 * @param pin
 * @param interrupt
 */
bool gpio_interrupt_configuration(GPIO_MemMapPtr gpio,uint32_t pin,gpio_interrupt_t interrupt)
{
	PORT_MemMapPtr port;

	if(gpio == GPIOA)
	{
		port = PORTA;
	}
	else if(gpio == GPIOB)
	{
		port = PORTB;
	}
	else
	{
		return false;
	}

	if(port == PORTA)
	{
		PORT_PCR_REG(PORTA_BASE_PTR,pin) |= PORT_PCR_IRQC(interrupt);
		PORT_PCR_REG(PORTA_BASE_PTR,pin) |= PORT_PCR_ISF_MASK; // clear flag
		// PORTB_PCR0 |= PORT_PCR_ISF_MASK; // clear flag
		NVIC_EnableIRQ(PORTA_IRQn);
	}
	else // PORTB
	{
		PORT_PCR_REG(PORTB_BASE_PTR,pin) |= PORT_PCR_IRQC(interrupt);
		PORT_PCR_REG(PORTB_BASE_PTR,pin) |= PORT_PCR_ISF_MASK; // clear flag
		NVIC_EnableIRQ(PORTB_IRQn);
	}

	return true;
}

/**
 * @brief
 * @param gpio
 * @param *task
 */
bool gpio_set_callback_irq(GPIO_MemMapPtr gpio, void (*task)(void))
{
	PORT_MemMapPtr port;

	if(gpio == GPIOA)
	{
		port = PORTA;
	}
	else if(gpio == GPIOB)
	{
		port = PORTB;
	}
	else
	{
		return false;
	}

	if(port == PORTA)
	{
		gpio_porta_irq = task;
	}
	else // PORTB
	{
		gpio_portb_irq = task;
	}

	return true;
}
/**
 * @brief
 */
void PORTA_IRQHandler(void)
{
    // Clear the interrupt flag
    PORTA_PCR12 |= PORT_PCR_ISF_MASK;
    PORTA_PCR13 |= PORT_PCR_ISF_MASK;
	//PORT_PCR_REG(PORTA_BASE_PTR,13) |= PORT_PCR_ISF_MASK; // clear flag
    if(gpio_porta_irq != NULL)
    {
    	gpio_porta_irq();
    }
}

/**
 * @brief
 */
void PORTB_IRQHandler(void)
{
    PORTB_PCR12 |= PORT_PCR_ISF_MASK;
    PORTB_PCR13 |= PORT_PCR_ISF_MASK;
	//PORT_PCR_REG(PORTB_BASE_PTR,12) |= PORT_PCR_ISF_MASK; // clear flag
    if(gpio_portb_irq != NULL)
    {
    	gpio_portb_irq();
    }
}

/*
 * uart.c
 *
 *  Created on: 23/07/2019
 *      Author: Evandro Teixeira
 */

#include "uart.h"

/**
 *
 */
void (*uart_irq)(void);

/**
 * @brief
 * @param uartch
 * @param pins
 * @param sysclk
 * @param baud
 * @return
 */
bool uart_init (UART0_MemMapPtr uartch,uart_pins_t pins, uint32_t sysclk, uint32_t baud)
{
    register uint16_t sbr;
    uint8_t temp;
    bool ret = false;

    if(uartch == UART0)
    {
		SIM_SCGC4 |= SIM_SCGC4_UART0_MASK;
		// UART0 clock source select
		// Selects the clock source for the UART0 transmit and receive clock.
		// 00 Clock disabled
		// 01 MCGFLLCLK clock
		// 10 OSCERCLK clock
		// 11 MCGIRCLK clock
		SIM_SOPT2 &= ~SIM_SOPT2_UART0SRC_MASK; 		// Clear SIM_SOPT2
		SIM_SOPT2 |=  SIM_SOPT2_UART0SRC(1);		// 01 MCGFLLCLK clock
		switch(pins)
		{
			case Option_Pins_0:
				SIM_SCGC5 |= SIM_SCGC5_PORTB_MASK;	// Turn on clock to B module
				PORTB_PCR1 = PORT_PCR_ISF_MASK | PORT_PCR_MUX(3);		// Set PTB1 to mux 3 [RX]
				PORTB_PCR2 = PORT_PCR_ISF_MASK | PORT_PCR_MUX(3);		// Set PTB2 to mux 3 [TX]
			break;
			case Option_Pins_1:
				SIM_SCGC5 |= SIM_SCGC5_PORTB_MASK;	// Turn on clock to B module
				PORTB_PCR3 = PORT_PCR_ISF_MASK | PORT_PCR_MUX(3);		// Set PTB3 to mux 3 [TX]
				PORTB_PCR4 = PORT_PCR_ISF_MASK | PORT_PCR_MUX(3);		// Set PTB4 to mux 3 [RX]
			default:
			break;
		}

		/* Make sure that the transmitter and receiver are disabled while we change settings.*/
		UART0_C2_REG(uartch) &= ~(UART0_C2_TE_MASK | UART0_C2_RE_MASK );

		/* Configure the uart for 8-bit mode, no parity */
		UART0_C1_REG(uartch) = 0;	/* We need all default settings, so entire register is cleared */

		/* Calculate baud settings */
		temp = UART0_C4;
		temp = (temp & UART0_C4_OSR_MASK) + 1;
		sbr = (uint16_t)((sysclk)/(baud * (temp)));

		/* Save off the current value of the uartx_BDH except for the SBR field */
		temp = UART0_BDH_REG(uartch) & ~(UART0_BDH_SBR(0x1F));

		UART0_BDH_REG(uartch) = temp |  UART0_BDH_SBR(((sbr & 0x1F00) >> 8));
		UART0_BDL_REG(uartch) = (uint8_t)(sbr & UART0_BDL_SBR_MASK);

		/* Enable receiver and transmitter */
		UART0_C2_REG(uartch) |= (UART0_C2_TE_MASK | UART0_C2_RE_MASK );

		ret = true;
    }
    else
    {
    	ret = false;
    }
    return ret;
}

/**
 * @brief
 * @param channel
 * @return
 */
char uart_getchar (UART0_MemMapPtr channel)
{
	/* Wait until character has been received */
	while (!(UART0_S1_REG(channel) & UART0_S1_RDRF_MASK));

	/* Return the 8-bit data from the receiver */
	return UART0_D_REG(channel);
}

/**
 * @brief
 * @param channel
 * @param ch
 */
void uart_putchar (UART0_MemMapPtr channel, char ch)
{
	/* Wait until space is available in the FIFO */
	while(!(UART0_S1_REG(channel) & UART0_S1_TDRE_MASK));

	/* Send the character */
	UART0_D_REG(channel) = (uint8_t)ch;
 }

/**
 * @brief
 * @param channel
 * @param *txt
 */
void uart_put_string(UART0_MemMapPtr channel, char *txt)
{
	while(*txt)
	{
		uart_putchar (channel,*txt);
		txt++;
	}
}

/**
 * @brief
 * @param channel
 */
bool uart_enable_irq(UART0_MemMapPtr channel)
{
	bool ret = false;
	if(channel == UART0)
	{
		UART0_C2_REG(channel) |= UART0_C2_RIE_MASK;
		//UART0_C2_REG(channel) |= UART0_C2_TIE_MASK;
		UART0_C2_REG(channel) |= UART0_C3_ORIE_MASK;
		NVIC_EnableIRQ(UART0_IRQn);
		ret = true;
	}
	return ret;
}

/**
 * @brief
 * @param channel
 * @param *task
 */
bool uart_set_callback_irq(UART0_MemMapPtr channel,void (*task)(void))
{
	bool ret = false;
	if((channel == UART0) && (task != NULL))
	{
		uart_irq = task;
		ret = true;
	}
	return ret;
}

/**
 *
 */
void UART0_IRQHandler(void)
{
	if(uart_irq != NULL)
	{
		uart_irq();
	}
}

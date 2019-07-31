/*
 * spi.c
 *
 *  Created on: 25/07/2019
 *      Author: Evandro Teixeira
 */

#include "spi.h"

/**
 * @brief
 * @param spi
 * @param type
 * @param alt
 * @param pre
 * @param div
 * @return
 */
bool spi_init(SPI_MemMapPtr spi,spi_type_t type,spi_alt_pin_t alt, uint8_t pre, uint8_t div)
{
	bool ret = true;
	switch(alt)
	{
		case SPI_Alt0:
			SIM_SCGC5 |= SIM_SCGC5_PORTA_MASK;  // Enable PORT
			//PORTA_PCR5 = PORT_PCR_MUX(0x3);	//	Set PTA5 to mux 3 [SPI0_SS  ]
			PORTA_PCR6 = PORT_PCR_MUX(0x3);		//	Set PTA6 to mux 3 [SPI0_MISO]
			PORTA_PCR7 = PORT_PCR_MUX(0x3);		//	Set PTA7 to mux 3 [SPI0_MOSI]
			PORTB_PCR0 = PORT_PCR_MUX(0x3);		//	Set PTB0 to mux 3 [SPI0_SCK ]
		break;
		case SPI_Alt1:
			SIM_SCGC5 |= SIM_SCGC5_PORTB_MASK;	// Enable PORT
			//PORTA_PCR19 = PORT_PCR_MUX(0x3);	//	Set PTA19 to mux 3 [SPI0_SS  ]
			PORTB_PCR15 = PORT_PCR_MUX(0x3);	//	Set PTB15 to mux 3 [SPI0_MISO]
			PORTB_PCR16 = PORT_PCR_MUX(0x3);	//	Set PTB16 to mux 3 [SPI0_MOSI]
			PORTB_PCR17 = PORT_PCR_MUX(0x3);	//	Set PTB17 to mux 3 [SPI0_SCK ]
		break;
		default:
			ret = false;
		break;
	}

	if(type == SPI_Master)
	{
		// enable clock gate for spi module
		SIM_SCGC4 |= SIM_SCGC4_SPI0_MASK;

		spi->C1 &= ~SPI_C1_SPE_MASK; // disable SPI
		// Set SPI to Master
		spi->C1 = SPI_C1_MSTR_MASK;
		// Configure SPI Register C2
		spi->C2 = SPI_C2_MODFEN_MASK;   //Master SS pin acts as slave select output
		// Set baud rate prescale divisor to 6 & set baud rate divisor to 4 for baud rate of 1 Mhz
		spi->BR = (SPI_BR_SPPR(pre) | SPI_BR_SPR(div));    //  Mhz
		// Enable SPI
		spi->C1 |= SPI_C1_SPE_MASK;
	}
	else if(type == SPI_Slave)
	{
		ret = false;
	}
	else
	{
		ret = false;
	}

	return ret;
}

/**
 * @brief
 * @param spi
 * @param data
 */
void spi_write(SPI_MemMapPtr spi, uint8_t data)
{
	while(!(SPI_S_SPTEF_MASK & spi->S))
	{
		__asm("nop");
	}
	spi->D = data;
}

/**
 * @brief
 * @param spi
 */
uint8_t spi_read(SPI_MemMapPtr spi)
{
	while (!(spi->S & SPI_S_SPTEF_MASK));
	{
		__asm("nop");
	}
	spi->D = 0;
	while(!(spi->S & SPI_S_SPRF_MASK))
	{
		__asm("nop");
	}
	return spi->D;
}

/**
 * @brief
 * @param gpio
 * @param pin
 * @param st
 */
void spi_write_CS(GPIO_MemMapPtr gpio,uint32_t pin,spi_cs_t st)
{
	gpio_write(gpio,pin,st);
}

/**
 * @brief
 * @param gpio
 * @param pin
 */
void spi_init_CS(GPIO_MemMapPtr gpio,uint32_t pin)
{
	gpio_init(gpio,pin,GPIO_OUTPUT);
	gpio_write(gpio,pin,SPI_CS_Disable);
}

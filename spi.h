/*
 * spi.h
 *
 *  Created on: 25/07/2019
 *      Author: Evandro Teixeira
 */

#ifndef KL05Z_LIBRARIES_SPI_H_
#define KL05Z_LIBRARIES_SPI_H_


#include "kl05_libraries.h"

typedef enum
{
	SPI_Alt0 = 0,
	SPI_Alt1,
	//SPI_Alt2,
}spi_alt_pin_t;

typedef enum
{
	SPI_Master = 0,
	SPI_Slave,
}spi_type_t;

typedef enum
{
	SPI_CS_Enable = 0,
	SPI_CS_Disable,
}spi_cs_t;


bool spi_init(SPI_MemMapPtr spi,spi_type_t type, spi_alt_pin_t alt, uint8_t pre, uint8_t div);
void spi_write(SPI_MemMapPtr spi, uint8_t data);
uint8_t spi_read(SPI_MemMapPtr spi);
void spi_write_CS(GPIO_MemMapPtr gpio,uint32_t pin,spi_cs_t st);
void spi_init_CS(GPIO_MemMapPtr gpio,uint32_t pin);

#endif /* KL05Z_LIBRARIES_SPI_H_ */

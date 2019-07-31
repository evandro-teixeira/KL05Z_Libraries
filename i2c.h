/*
 * i2c.h
 *
 *  Created on: 26/07/2019
 *      Author: Evandro Teixeira
 */

#ifndef KL05Z_LIBRARIES_I2C_H_
#define KL05Z_LIBRARIES_I2C_H_

#include "kl05_libraries.h"

typedef enum
{
	Alt_Pins_0 = 0,
	Alt_Pins_1,
}i2c_alt_pins_t;

typedef enum
{
	Mult_1 = 0,
	Mult_2,
	Mult_4,
}i2c_mult_t;

bool i2c_init(I2C_MemMapPtr i2c, i2c_alt_pins_t alt, i2c_mult_t mult, uint8_t icr);
void i2c_send_data(I2C_MemMapPtr i2c, uint8_t address, uint8_t reg, uint8_t data);
void i2c_send_buffer(I2C_MemMapPtr i2c, uint8_t address, uint8_t reg, uint8_t *buffer);
uint8_t i2c_read_data(I2C_MemMapPtr i2c, int8_t address, uint8_t reg);
void i2c_read_buffer(I2C_MemMapPtr i2c, int8_t address, uint8_t reg, uint8_t nbyte, uint8_t *buffer);

#endif /* KL05Z_LIBRARIES_I2C_H_ */

/*
 * i2c.c
 *
 *  Created on: 26/07/2019
 *      Author: Evandro Teixeira
 */

#include "i2c.h"

/*
 * @brief
 */
static uint8_t error;
static uint16_t timeout;

/**
 * @brief
 */
uint8_t i2c_start(I2C_MemMapPtr i2c);
uint8_t i2c_stop(I2C_MemMapPtr i2c);
uint8_t i2c_repeat_start(I2C_MemMapPtr i2c);
void i2c_wait(I2C_MemMapPtr i2c);
void i2c_delay(void);

uint8_t i2c_cycle_read(I2C_MemMapPtr i2c, uint8_t ack);
uint8_t i2c_cycle_write(I2C_MemMapPtr i2c, uint8_t data);
/**
 * @brief
 * @param i2c
 * @param alt
 * @param mult
 * @param icr
 */
bool i2c_init(I2C_MemMapPtr i2c, i2c_alt_pins_t alt, i2c_mult_t mult, uint8_t icr)
{
	switch(alt)
	{
		case Alt_Pins_0:
		    SIM_SCGC5 |= SIM_SCGC5_PORTB_MASK; // Enable PORTB Clock
		    PORTB_PCR3 = PORT_PCR_MUX(2);
		    PORTB_PCR4 = PORT_PCR_MUX(2);
		break;
		case Alt_Pins_1:
		    SIM_SCGC5 |= SIM_SCGC5_PORTA_MASK; // Enable PORTA Clock
		    PORTA_PCR3 = PORT_PCR_MUX(2);
		    PORTA_PCR4 = PORT_PCR_MUX(2);
		break;
		default:
			return false;
		break;
	}
	SIM_SCGC4 |= SIM_SCGC4_I2C0_MASK; 			// Enable IIC0 clock
	i2c->F = I2C_F_ICR(icr) | I2C_F_MULT(mult);	// Config Clock
	i2c->C1 = 0; 								// Clear
	i2c->C1 = I2C_C1_IICEN_MASK/*Enable IIC*/ | I2C_C1_MST_MASK/*Master*/ | I2C_C1_TX_MASK/*Transmit*/;

	return true;
}

/**
 * @brief Initiate I2C Start Condition
 * @param i2c
 */
uint8_t i2c_start(I2C_MemMapPtr i2c)
{
  error = 0x00;
  i2c->C1 |= I2C_C1_TX_MASK;
  i2c->C1 |= I2C_C1_MST_MASK;
  timeout = 0;
  while ((!(i2c->S & I2C_S_BUSY_MASK)) && (timeout<10000))
    timeout++;
  if (timeout >= 10000)
    error |= 0x01;
  return error; // Wait until BUSY=1
}

/**
 * @brief Initiate I2C Stop Condition
 * @param i2c
 * @return Wait until BUSY=0
 */
uint8_t i2c_stop(I2C_MemMapPtr i2c)
{
  error = 0x00;
  i2c->C1 &= ~I2C_C1_MST_MASK;
  timeout = 0;
  while ( (i2c->S & I2C_S_BUSY_MASK) && (timeout<50000))
    timeout++;
  if (timeout >= 50000)
    error |= 0x02;
  return error; // Wait until BUSY=0
}

/**
 * @brief Initiate I2C Repeat Start Condition
 * @param i2c
 * @return Wait until BUSY=1
 */
uint8_t i2c_repeat_start(I2C_MemMapPtr i2c)
{
  error = 0x00;
  i2c->C1 |= I2C_C1_RSTA_MASK;
  timeout = 0;
  while ((!(I2C0_S & I2C_S_BUSY_MASK)) && (timeout<10000))
    timeout++;
  if (timeout >= 10000)
    error |= 0x04;

  return error; // Wait until BUSY=1
}

/**
 * @brief
 * @param
 */
void i2c_wait(I2C_MemMapPtr i2c)
{
	uint32_t i = 100;
	while(((i2c->S & I2C_S_IICIF_MASK) == 0) && i)
	{
		i--;
	}
	i2c->S |= I2C_S_IICIF_MASK;
}

/**
 * @brief i2c delay
 */
void i2c_Delay(void)
{
  uint8_t I2Cd;
  for (I2Cd=0; I2Cd<100; I2Cd++);
}


/**
 * @brief I2C Cycle Write
 * @param data
 */
uint8_t i2c_cycle_write(I2C_MemMapPtr i2c, uint8_t data)
{
  timeout = 0;
  error = 0x00;
  while ((!(i2c->S & I2C_S_TCF_MASK)) && (timeout<10000))
    timeout++;
  if (timeout >= 10000)
    error |= 0x08;
  i2c->C1 |= I2C_C1_TX_MASK;
  i2c->D = data;
  timeout = 0;
  while ((!(i2c->S & I2C_S_IICIF_MASK)) && (timeout<10000))
    timeout++;
  if (timeout >= 10000)
    error |= 0x10;
  i2c->S |= I2C_S_IICIF_MASK;    // clear the int pending flag
  if (i2c->S & I2C_S_RXAK_MASK)  // no ack received
    error |= 0x20;
  return error;
}


/**
 * @brief I2C Cycle Read
 * @param i2c
 * @param ack
 * @return
 */
uint8_t i2c_cycle_read(I2C_MemMapPtr i2c, uint8_t ack)
{
  uint8_t bread;
  timeout = 0;
  error = 0x00;

  while ((!(i2c->S & I2C_S_TCF_MASK)) && (timeout<10000))
    timeout++;
  if (timeout >= 10000)
    error|=0x08;
  i2c->C1 &= ~I2C_C1_TX_MASK;     // Receive mode
  if( ack )
  {
	  i2c->C1 |= I2C_C1_TXAK_MASK;
  }
  else
  {
	  i2c->C1 &= ~I2C_C1_TXAK_MASK;
  }
  bread = i2c->D;
  timeout = 0;
  while ((!(i2c->S & I2C_S_IICIF_MASK)) && (timeout<10000))
    timeout++;
  if (timeout >= 10000)
    error |= 0x10;
  i2c->S &= I2C_S_IICIF_MASK;    // clear the int pending flag

  return bread;
}

/**
 * @brief
 * @param i2c
 * @param address
 * @param reg
 * @param data
 */
void i2c_send_data(I2C_MemMapPtr i2c, uint8_t address, uint8_t reg, uint8_t data)
{
	kl05z_disable_interrupts();
	i2c->C1 |= I2C_C1_TX_MASK;
	i2c_start(i2c);                                  // Send Start
	i2c_cycle_write(i2c, (address << 1) | 0 );       // Send I2C "Write" Address
	i2c_cycle_write(i2c, reg);                       // Send Register
	i2c_cycle_write(i2c, data);                      // Send Value
	i2c_stop(i2c);                                   // Send Stop
	kl05z_enable_interrupts();
}

/**
 * @brief
 * @param i2c
 * @param address
 * @param reg
 * @param *buffer
 */
void i2c_send_buffer(I2C_MemMapPtr i2c, uint8_t address, uint8_t reg, uint8_t *buffer)
{
	uint32_t size_byte = 0;
	size_byte = strlen(buffer);

	kl05z_disable_interrupts();
	i2c->C1 |= I2C_C1_TX_MASK;
	i2c_start(i2c);                                  // Send Start
	i2c_cycle_write(i2c, (address << 1) | 0 );                      // Send I2C "Write" Address
	i2c_cycle_write(i2c, reg);                         // Send Register
	while (size_byte)                             // Send N bytes
	{
		i2c_cycle_write(i2c, *buffer);
		buffer++;
		size_byte--;
	}
	i2c_stop(i2c);                                   // Send Stop
	kl05z_enable_interrupts();
}

/**
 * @brief
 * @param i2c
 * @param address
 * @param reg
 * @return
 */
uint8_t i2c_read_data(I2C_MemMapPtr i2c, int8_t address, uint8_t reg)
{
	uint8_t b;
	kl05z_disable_interrupts();
	i2c->C1 |= I2C_C1_TX_MASK;
	i2c_start(i2c);                                  // Send Start
	i2c_cycle_write(i2c, (address << 1) | 0);           // Send I2C "Write" Address
	i2c_cycle_write(i2c, reg);                          // Send Register
	i2c_repeat_start(i2c);                            // Send Repeat Start
	i2c_cycle_write(i2c, (address << 1) | 1);           // Send I2C "Read" Address
	i2c->C1 &=~ I2C_C1_TX_MASK;
	i2c->C1 |= I2C_C1_TXAK_MASK;
	b = i2c_cycle_read(i2c, 1);                         // *** Dummy read: reads "I2C_ReadAddress" value ***
	b = i2c_cycle_read(i2c, 1);                         // Read Register Value
	i2c_stop(i2c);                                   // Send Stop
	kl05z_enable_interrupts();
	return b;
}

/**
 * @brief
 * @param i2c
 * @param address
 * @param reg
 * @param nbyte
 * @param *buffer
 */
void i2c_read_buffer(I2C_MemMapPtr i2c, int8_t address, uint8_t reg, uint8_t nbyte, uint8_t *buffer)
{
	uint8_t b;
	kl05z_disable_interrupts();
	i2c->C1 |= I2C_C1_TX_MASK;
	i2c_start(i2c);                                  // Send Start
	i2c_cycle_write(i2c, address);                      // Send I2C "Write" Address
	i2c_cycle_write(i2c, reg);                          // Send Register
	i2c_repeat_start(i2c);                            // Send Repeat Start
	i2c_cycle_write(i2c, address+1);                    // Send I2C "Read" Address
	b = i2c_cycle_read(i2c, 0);                         // *** Dummy read: reads "I2C_ReadAddress" value ***
	while (nbyte > 1)                             // Read N-1 Register Values
	{
		b = i2c_cycle_read(i2c, 0);
		*buffer = b;
		buffer++;
		nbyte--;
	}
	b = i2c_cycle_read(i2c, 1);
	*buffer = b;                                   // Read Last value
	i2c_stop(i2c);                                   // Send Stop
	kl05z_enable_interrupts();
}

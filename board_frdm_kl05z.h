/*
 * board_frdm_kl05z.h
 *
 *  Created on: 22/07/2019
 *      Author: Evandro Teixeira
 */

#ifndef BOARD_FRDM_KL05Z_H_
#define BOARD_FRDM_KL05Z_H_

#include "kl05_libraries.h"

/*  */
#define D0				GPIOB,2			// PTB2
#define D1				GPIOB,1			// PTB1
#define D2				GPIOA,11		// PTA11
#define D3				GPIOB,5			// PTA11
#define D4				GPIOA,10		// PTA10
#define D5				GPIOA,12		// PTA12
#define D6				GPIOB,6			// PTB6
#define D7				GPIOB,7 		// PTB7
#define D8				GPIOB,10		// PTB10
#define D9				GPIOB,11		// PTB11
#define D10				GPIOA,5 		// PTA5
#define D11				GPIOA,7			// PTA7
#define D12				GPIOA,6			// PTA6
#define D13				GPIOB,0			// PTB0
#define D14				GPIOB,4			// PTB4
#define D15				GPIOB,3			// PTB3

/* */
#define A0				GPIOB,8			// PTB8
#define A1				GPIOB,9			// PTB9
#define A2				GPIOA,8			// PTA8
#define A3				GPIOA,0			// PTA0
#define A4				GPIOA,9			// PTA9
#define A5				GPIOB,13		// PTB13

/* */
#define LED_RED			GPIOB,8			// PTB8 - LED RGB RED
#define LED_GREEN		GPIOB,9			// PTB9 - LED RGB GREEN
#define LED_BLUE		GPIOB,10		// PTB10 - LED RGB BLUE

/* */		
#define I2C_CONFIG 		I2C0,Alt_Pins_0,Mult_1,16 	// I2C CLK 180 KHz - PTB3 PTB4
#define I2C 			I2C0

/* */
#define SPI_CONFIG 		SPI0,SPI_Master,SPI_Alt0,1,4	// SPI CLK 180 KHz - PTA6 PTA7 PTB0
#define SPI 			SPI0

/* */
#define UART_CONFIG		UART0,Option_Pins_0,SystemCoreClock // UART - PTB1 PTB2
#define UART 			UART0


#endif /* BOARD_FRDM_KL05Z_H_ */

/*
 * kl05_libraries.h
 *
 *  Created on: 22/07/2019
 *      Author: Evandro Teixeira
 */

#ifndef KL05Z_LIBRARIES_KL05_LIBRARIES_H_
#define KL05Z_LIBRARIES_KL05_LIBRARIES_H_


#include "MKL05Z4.h"
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#ifndef NULL
#define NULL ((void *)0)
#endif

#define kl05z_disable_interrupts() 	__disable_irq()
#define kl05z_enable_interrupts()  	__enable_irq()

#define kl05z_enable_irq(x) 		NVIC_EnableIRQ(x)
#define kl05z_disable_irq(x) 		NVIC_DisableIRQ(x)

#include "gpio.h"
#include "uart.h"
#include "pit.h"
#include "spi.h"
#include "systick.h"

#endif /* KL05Z_LIBRARIES_KL05_LIBRARIES_H_ */

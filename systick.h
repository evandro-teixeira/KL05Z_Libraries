/*
 * systick.h
 *
 *  Created on: 25/07/2019
 *      Author: Evandro Teixeira
 */

#ifndef KL05Z_LIBRARIES_SYSTICK_H_
#define KL05Z_LIBRARIES_SYSTICK_H_

#include "kl05_libraries.h"

void systick_init(void);
uint32_t systick_get_ticks(void);
void systick_delay(uint32_t delay);
void systick_suspend_ticks(void);
void systick_resume_ticks(void);

#endif /* KL05Z_LIBRARIES_SYSTICK_H_ */

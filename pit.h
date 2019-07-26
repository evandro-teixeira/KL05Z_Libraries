/*
 * pit.h
 *
 *  Created on: 25/07/2019
 *      Author: Evandro Teixeira
 */

#ifndef KL05Z_LIBRARIES_PIT_H_
#define KL05Z_LIBRARIES_PIT_H_

#include "kl05_libraries.h"

#define PIT_NUMBER_CHANNEL	2
#define PIT_NUMBER_INDEX	2

#ifndef irqTask
typedef void(*irqTask)(void);
#endif

typedef enum
{
	PIT_CHANNEL_0 = 0,
	PIT_CHANNEL_1,
}pit_channel_t;

bool pit_init(uint32_t value,pit_channel_t ch);
void pit_start(pit_channel_t ch);
void pit_stop(pit_channel_t ch);
bool pit_set_callback_irq(pit_channel_t ch, void (*task_irq)(void));

#endif /* KL05Z_LIBRARIES_PIT_H_ */

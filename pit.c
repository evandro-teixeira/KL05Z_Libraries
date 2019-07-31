/*
 * pit.c
 *
 *  Created on: 25/07/2019
 *      Author: Evandro Teixeira
 */

#include "pit.h"

/**
 * @brief
 */
irqTask pit_irq[PIT_NUMBER_CHANNEL][PIT_NUMBER_INDEX] = {NULL};
static uint8_t index_ch[PIT_NUMBER_INDEX] = {0};

/**
 * @brief
 * @param valeu
 * @param ch
 * @return
 */
bool pit_init(uint32_t value,pit_channel_t ch)
{
	if(ch < PIT_NUMBER_INDEX)
	{
		// Enable PIT clock
		SIM_SCGC6 |= SIM_SCGC6_PIT_MASK;

		// Turn on PIT
		PIT_MCR = 0;

		// Configure PIT to produce an interrupt every 1s
		//PIT_LDVAL0 = value;
		PIT_LDVAL_REG(PIT,ch) = value;

		// Enable interrupt and enable timer
		//PIT_TCTRL0 |= PIT_TCTRL_TIE_MASK | PIT_TCTRL_TEN_MASK;
		PIT_TCTRL_REG(PIT,ch) |= PIT_TCTRL_TIE_MASK | PIT_TCTRL_TEN_MASK;

		PIT_TCTRL_REG(PIT,ch) &= ~PIT_TCTRL_TEN_MASK;

		// Enable External Interrupt
		NVIC_EnableIRQ(PIT_IRQn);

		return true;
	}
	else
	{
		return false;
	}
}

/**
 * @brief
 * @param
 */
void pit_start(pit_channel_t ch)
{
	PIT_TCTRL_REG(PIT,ch) |= PIT_TCTRL_TEN_MASK;
}

/**
 * @brief
 * @param
 */
void pit_stop(pit_channel_t ch)
{
	PIT_TCTRL_REG(PIT,ch) &= ~PIT_TCTRL_TEN_MASK;
}

/**
 * @brief
 * @param ch
 * @param *task_irq
 */
bool pit_set_callback_irq(pit_channel_t ch, void (*task_irq)(void))
{
	if(index_ch[ch] < PIT_NUMBER_INDEX)
	{
		pit_irq[ch][ (index_ch[ch]) ] = task_irq;
		index_ch[ch]++;
		return true;
	}
	else
	{
		return false;
	}
}

/**
 * @brief
 */
void PIT_IRQHandler(void)
{
	uint8_t index = 0;
	uint8_t i = 0;

	for(index=0;index<PIT_NUMBER_CHANNEL;index++)
	{
		if( PIT_TFLG_REG(PIT,index) )
		{
			PIT_TFLG_REG(PIT,index) = PIT_TFLG_TIF_MASK;
			for(i=0;i<index_ch[index];i++)
			{
				pit_irq[index][i]();
			}
		}
	}
}

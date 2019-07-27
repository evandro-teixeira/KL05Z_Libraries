/*
 * systick.c
 *
 *  Created on: 25/07/2019
 *      Author: Evandro Teixeira
 */

#include "systick.h"

/**
 *
 */
static uint32_t ticks = 0;

/**
 * @brief
 */
void systick_init(void)
{
//	SysTick->LOAD  = (SystemCoreClock/(SystemCoreClock/10000)) + 1;	// periodo de 1ms
//	SysTick->VAL   = 0;
//	SysTick->CTRL  = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_TICKINT_Msk | SysTick_CTRL_ENABLE_Msk;

	SysTick_Config(SystemCoreClock / 1000);
	NVIC_EnableIRQ(SysTick_IRQn);
}

/**
 *
 */
uint32_t systick_get_ticks(void)
{
	return ticks;
}

/**
 *
 */
void systick_delay(uint32_t delay)
{
	uint32_t ticks_start = systick_get_ticks();
	if(delay < 0xFFFFFFFFU)
	{
		delay++;
	}

	while((systick_get_ticks() - ticks_start) < delay) {}
}

/**
 *
 */
void systick_suspend_ticks(void)
{
	/* Disable SysTick Interrupt */
	CLEAR_BIT(SysTick->CTRL,SysTick_CTRL_TICKINT_Msk);
}

/**
 *
 */
void systick_resume_ticks(void)
{
	/* Enable SysTick Interrupt */
	SET_BIT(SysTick->CTRL,SysTick_CTRL_TICKINT_Msk);
}

/**
 *
 */
void SysTick_Handler(void)
{
	ticks++;
}



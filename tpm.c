/*
 * tpm.c
 *
 *  Created on: 26/07/2019
 *      Author: Evandro Teixeira
 */

#include "tpm.h"



#define MAX_TIMER_TICK		(uint16_t)(65535)
#define US_CONVERSION		(uint32_t)(1000000)

/**
 *
 */
void (*tpm0_timer_task_irq)(void);
void (*tpm1_timer_task_irq)(void);

/**
 * @brief
 * @param tpm
 * @param sysck
 * @param time_us
 * @param *task
 * @return
 */
bool tpm_timer_init(TPM_MemMapPtr tpm, uint32_t sysck, uint32_t time_us, void (*task)(void))
{
	uint8_t divide[8] = {1,2,4,8,16,32,64,128};
	uint32_t count = 0;
	uint32_t prescaler = 0;
	uint32_t clk = 0;

	if(tpm == TPM0)
	{
		if(task != NULL)
		{
			tpm0_timer_task_irq = task;
			SIM_SCGC6 |= SIM_SCGC6_TPM0_MASK;
			NVIC_EnableIRQ(TPM0_IRQn);
		}
		else
		{
			return false;
		}
	}
	else if(tpm == TPM1)
	{
		if(task != NULL)
		{
			tpm1_timer_task_irq = task;
			SIM_SCGC6 |= SIM_SCGC6_TPM1_MASK;
			NVIC_EnableIRQ(TPM1_IRQn);
		}
		else
		{
			return false;
		}
	}
	else
	{
		return false;
	}

	// Calcula numero de tickis e valor prescales
	do
	{
		clk = (uint32_t)((float)sysck / divide[prescaler]);
		count = (uint32_t)((uint64_t)time_us * clk / US_CONVERSION);
		if(count > MAX_TIMER_TICK)
		{
			if(prescaler < 8)
			{
				prescaler++;
			}
			else
			{
				return false;
			}
		}
	}while(count > MAX_TIMER_TICK);

	SIM_SOPT2  |= SIM_SOPT2_TPMSRC(1);
	tpm->CNT 	= TPM_CNT_COUNT(0);
	tpm->MOD	= TPM_MOD_MOD(count);
	tpm->SC 	= TPM_SC_TOIE_MASK|/*TPM_SC_CMOD(1)|*/TPM_SC_PS(prescaler);
	return true;
}

/**
 * @brief
 * @param tpm
 */
void tpm_timer_start(TPM_MemMapPtr tpm)
{
	tpm->SC |= TPM_SC_CMOD(1);
}

/**
 * @brief
 * @param tpm
 */
void tpm_timer_stop(TPM_MemMapPtr tpm)
{
	tpm->SC |= TPM_SC_CMOD(0);
}

/**
 * @brief
 */
void TPM0_IRQHandler(void)
{
	// Clear IRQ
	TPM0->SC |= TPM_SC_TOF_MASK;
	TPM0->CNT = TPM_CNT_COUNT(0);

	if(tpm0_timer_task_irq != NULL)
	{
		tpm0_timer_task_irq();
	}
	// Restart IRQ
	TPM0->SC |= TPM_SC_CMOD(1);
}

/**
 * @brief
 */
void TPM1_IRQHandler(void)
{
	// Clear IRQ
	TPM1->SC |= TPM_SC_TOF_MASK;
	TPM1->CNT = TPM_CNT_COUNT(0);

	if(tpm1_timer_task_irq != NULL)
	{
		tpm1_timer_task_irq();
	}
	// Restart IRQ
	TPM1->SC |= TPM_SC_CMOD(1);
}

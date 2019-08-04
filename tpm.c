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
		SIM_SCGC6 |= SIM_SCGC6_TPM0_MASK;
		NVIC_EnableIRQ(TPM0_IRQn);
	}
	else if(tpm == TPM1)
	{
		SIM_SCGC6 |= SIM_SCGC6_TPM1_MASK;
		NVIC_EnableIRQ(TPM1_IRQn);
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
 * @param *task
 * @return
 */
bool tpm_timer_set_callback_irq(TPM_MemMapPtr tpm, void (*task)(void))
{
	if((tpm == TPM0) && (task != NULL))
	{
		tpm0_timer_task_irq = task;
	}
	else if((tpm == TPM1) && (task != NULL))
	{
		tpm1_timer_task_irq = task;
	}
	else
	{
		return false;
	}

	return true;
}

/**
 * @brief
 * @param tpm
 * @param sysck
 * @param freq
 * @param counting_mode
 * @return
 */
bool tpm_pwm_init(TPM_MemMapPtr tpm, uint32_t sysck, uint32_t freq,bool counting_mode)
{
	uint8_t divide[8] = {1,2,4,8,16,32,64,128};
	uint32_t count = 0;
	uint32_t prescaler = 0;
	uint32_t clk = 0;

	if(tpm == TPM0)
	{
		SIM_SCGC6 |= SIM_SCGC6_TPM0_MASK;
	}
	else if(tpm == TPM1)
	{
		SIM_SCGC6 |= SIM_SCGC6_TPM1_MASK;
	}
	else
	{
		return false;
	}

	// Calcula numero de tickis e valor prescales
	do
	{
		clk = (uint32_t)((float)sysck / divide[prescaler]);
		count = (uint32_t)((uint64_t)(1/freq) * clk / US_CONVERSION);
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

	SIM_SOPT2 |= SIM_SOPT2_TPMSRC(clk);
	tpm->MOD = TPM_MOD_MOD(count);
	tpm->SC |= TPM_SC_CMOD(1) | TPM_SC_PS(prescaler);

	if(counting_mode == TPM_CENTER_PWM)
	{
		tpm->SC |= TPM_SC_CPWMS_MASK;
	}
	else if(counting_mode == TPM_EDGE_PWM)
	{
		tpm->SC &= ~TPM_SC_CPWMS_MASK;
	}
	else
	{
		return false;
	}
	return true;
}

/**
 * @brief
 * @param tpm
 * @param channel
 * @param mode
 * @param gpio
 * @param pin
 * @note: 	Pin PWM
 * 			PTB6 	TPM0 	CH3
 * 			PTB8 	TPM0 	CH3
 * 			PTB7 	TPM0 	CH2
 * 			PTB9 	TPM0 	CH2
 * 			PTA5 	TPM0 	CH5
 * 			PTA6 	TPM0 	CH4
 * 			PTB10 	TPM0 	CH1
 * 			PTB11 	TPM0 	CH0
 *
 * 			PTA0 	TPM1 	CH0
 * 			PTA12 	TPM1 	CH0
 * 			PTB13 	TPM1 	CH1
 * 			PTB5 	TPM1 	CH1
 * @return
 */
bool tpm_pwm_init_channel(TPM_MemMapPtr tpm,tpm_pwm_channel_t channel, uint8_t mode,GPIO_MemMapPtr gpio,uint8_t pin)
{
	if(tpm == TPM0)
	{
		if(channel == tpm_pwm_channel_0)		// Channel 0 - PTB11
		{
			if((gpio == GPIOB) && (pin == 11))
			{
				SIM_SCGC5 |= SIM_SCGC5_PORTB_MASK;
				PORT_PCR_REG(PORTB_BASE_PTR,pin) = PORT_PCR_MUX(3);
			}
			else
			{
				return false;
			}
		}
		else if (channel == tpm_pwm_channel_1)	// channel 1 - PTB10
		{
			if((gpio == GPIOB) && (pin == 10))
			{
				SIM_SCGC5 |= SIM_SCGC5_PORTB_MASK;
				PORT_PCR_REG(PORTB_BASE_PTR,pin) = PORT_PCR_MUX(3);
			}
			else
			{
				return false;
			}
		}
		else if (channel == tpm_pwm_channel_2)	// channel 2 - PTB7 or PTB9
		{
			if((gpio == GPIOB) && ((pin == 9)||(pin == 7)))
			{
				SIM_SCGC5 |= SIM_SCGC5_PORTB_MASK;
				PORT_PCR_REG(PORTB_BASE_PTR,pin) = PORT_PCR_MUX(3);
			}
			else
			{
				return false;
			}
		}
		else if (channel == tpm_pwm_channel_3)	// channel 3 - PTB6 or PTB8
		{
			if((gpio == GPIOB) && ((pin == 6)||(pin == 8)))
			{
				SIM_SCGC5 |= SIM_SCGC5_PORTB_MASK;
				PORT_PCR_REG(PORTB_BASE_PTR,pin) = PORT_PCR_MUX(3);
			}
			else
			{
				return false;
			}
		}
		else if (channel == tpm_pwm_channel_4)	// channel - PTA6
		{
			if((gpio == GPIOA) && (pin == 6))
			{
				SIM_SCGC5 |= SIM_SCGC5_PORTA_MASK;
				PORT_PCR_REG(PORTA_BASE_PTR,pin) = PORT_PCR_MUX(3);
			}
			else
			{
				return false;
			}
		}
		else if (channel == tpm_pwm_channel_5)	// channel - PTA5
		{
			if((gpio == GPIOA) && (pin == 5))
			{
				SIM_SCGC5 |= SIM_SCGC5_PORTA_MASK;
				PORT_PCR_REG(PORTA_BASE_PTR,pin) = PORT_PCR_MUX(3);
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

		SIM_SCGC6 |= SIM_SCGC6_TPM0_MASK;

	}
	else if(tpm == TPM1)
	{
		if(channel == tpm_pwm_channel_0)		// channel 0 PTA0 or PTA12
		{
			if((gpio == GPIOA) && ((pin == 0)||(pin == 12)) )
			{
				SIM_SCGC5 |= SIM_SCGC5_PORTA_MASK;
				PORT_PCR_REG(PORTA_BASE_PTR,pin) = PORT_PCR_MUX(3);
			}
			else
			{
				return false;
			}
		}
		else if(channel == tpm_pwm_channel_1)	// channel 1 PTB5 or PTB13
		{
			if((gpio == GPIOB) && ((pin == 5)||(pin == 13)))
			{
				SIM_SCGC5 |= SIM_SCGC5_PORTB_MASK;
				PORT_PCR_REG(PORTB_BASE_PTR,pin) = PORT_PCR_MUX(3);
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

		SIM_SCGC6 |= SIM_SCGC6_TPM1_MASK;
	}
	else
	{
		return false;
	}

	TPM_CnSC_REG(tpm, channel) |= mode;

	return true;
}

/**
 * @brief
 * @param tpm
 * @param channel
 * @param value
 */
void tpm_pwm_duty_cycle(TPM_MemMapPtr tpm, uint16_t channel, uint16_t value)
{
	TPM_CnV_REG(tpm, channel) = value;
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

/*
 * tpm.h
 *
 *  Created on: 26/07/2019
 *      Author: Evandro Teixeira
 */

#ifndef KL05Z_LIBRARIES_TPM_H_
#define KL05Z_LIBRARIES_TPM_H_

#include "kl05_libraries.h"

typedef enum
{
	tpm_pwm_channel_0 = 0,
	tpm_pwm_channel_1,
	tpm_pwm_channel_2,
	tpm_pwm_channel_3,
	tpm_pwm_channel_4,
	tpm_pwm_channel_5,
	tpm_pwm_channel_NC
}tpm_pwm_channel_t;


bool tpm_timer_init(TPM_MemMapPtr tpm, uint32_t sysck, uint32_t time_us, void (*task)(void));
void tpm_timer_start(TPM_MemMapPtr tpm);
void tpm_timer_stop(TPM_MemMapPtr tpm);
bool tpm_timer_set_callback_irq(TPM_MemMapPtr tpm, void (*task)(void));

//bool tpm_pwm_init_channel(TPM_MemMapPtr tpm,tpm_pwm_channel_t channel, uint8_t mode,GPIO_MemMapPtr gpio,uint8_t pin);

#endif /* KL05Z_LIBRARIES_TPM_H_ */

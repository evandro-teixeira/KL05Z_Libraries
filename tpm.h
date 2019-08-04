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

#define TPM_PWM_H 		TPM_CnSC_MSB_MASK|TPM_CnSC_ELSB_MASK
#define TPM_PWM_L		TPM_CnSC_MSB_MASK|TPM_CnSC_ELSA_MASK

#define TPM_EDGE_PWM		0
#define TPM_CENTER_PWM		1


bool tpm_timer_init(TPM_MemMapPtr tpm, uint32_t sysck, uint32_t time_us, void (*task)(void));
void tpm_timer_start(TPM_MemMapPtr tpm);
void tpm_timer_stop(TPM_MemMapPtr tpm);
bool tpm_timer_set_callback_irq(TPM_MemMapPtr tpm, void (*task)(void));

bool tpm_pwm_init(TPM_MemMapPtr tpm, uint32_t sysck, uint32_t freq,bool counting_mode);
bool tpm_pwm_init_channel(TPM_MemMapPtr tpm,tpm_pwm_channel_t channel, uint8_t mode,GPIO_MemMapPtr gpio,uint8_t pin);
void tpm_pwm_duty_cycle(TPM_MemMapPtr tpm, uint16_t channel, uint16_t value);


#endif /* KL05Z_LIBRARIES_TPM_H_ */

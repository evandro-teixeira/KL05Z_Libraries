/*
 * tpm.h
 *
 *  Created on: 26/07/2019
 *      Author: Evandro Teixeira
 */

#ifndef KL05Z_LIBRARIES_TPM_H_
#define KL05Z_LIBRARIES_TPM_H_

#include "kl05_libraries.h"

bool tpm_timer_init(TPM_MemMapPtr tpm, uint32_t sysck, uint32_t time_us, void (*task)(void));
void tpm_timer_start(TPM_MemMapPtr tpm);
void tpm_timer_stop(TPM_MemMapPtr tpm);


#endif /* KL05Z_LIBRARIES_TPM_H_ */

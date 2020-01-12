#include "devSG90.h"
#include "SEGGER_RTT.h"
#include "gpio_pins.h"

volatile tpm_pwm_param_t SG90_PwmParam = 
{
	.mode = kTpmEdgeAlignedPWM,
	.edgeMode = kTpmHighTrue,
	.uFrequencyHZ = 50,
	.uDutyCyclePercent = 15,
};

void initSG90()
{
	SEGGER_RTT_WriteString(0, "\r\nInitialising SG90");

	tpm_general_config_t cfg = {
		false /*isDBGMode*/, 
		false /*isGlobalTimeBase*/,
	       	false /* isTriggerMode*/, 
		false /* isStopCountOnOverflow */, 
		false /* isCountReloadOnTrig */,
		0 /* triggerSource */
	};


	SEGGER_RTT_WriteString(0, "\r\n\tInitialising TPM\n");
	TPM_DRV_Init(tpm_instance, &cfg);
	SEGGER_RTT_WriteString(0, "\r\n\tSetting Clock");
	TPM_DRV_SetClock(tpm_instance, kTpmClockSourceModuleMCGIRCLK, kTpmDividedBy4);
	SEGGER_RTT_WriteString(0, "\r\n\tStarting PWM");
	
	bool pwminitsuccess = TPM_DRV_PwmStart(tpm_instance, (tpm_pwm_param_t *) &SG90_PwmParam, tpm_channel);

	SEGGER_RTT_printf(0, "\r\n\tPWM init success?  %d \r\n", pwminitsuccess);
	PORT_HAL_SetMuxMode(PORTB_BASE, 13u, kPortMuxAlt2);
}

void DeinitSG90()
{
	TPM_DRV_Deinit(tpm_instance);
}

void setSG90Position(uint32_t dutycycle)
{
	SG90_PwmParam.uDutyCyclePercent = dutycycle;
	while (!TPM_DRV_PwmStart(tpm_instance, (tpm_pwm_param_t *)&SG90_PwmParam, tpm_channel)){
		SEGGER_RTT_printf(0, "PWM FAILED TPM%dCH%d\n", tpm_instance, tpm_channel);
	}
}

void readSG90Registers()
{
}

void printTpmPwmParams(tpm_pwm_param_t * params)
{
	SEGGER_RTT_printf(0, "\r\nPWM params at address %08x", params);
	SEGGER_RTT_printf(0, "\r\n\t mode: %d", params->mode);
	SEGGER_RTT_printf(0, "\r\n\t edgemode: %d", params->edgeMode);
	SEGGER_RTT_printf(0, "\r\n\t uFrequencyHZ: %d", params->uFrequencyHZ);
	SEGGER_RTT_printf(0, "\r\n\t uDutyCyclePercent: %d", params->uDutyCyclePercent);
}

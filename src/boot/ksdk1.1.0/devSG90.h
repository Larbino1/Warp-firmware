/* Driver for the servo */

#ifndef WARP_BUILD_ENABLE_DEVSG90
#define WARP_BUILD_ENABLE_DEVSG90
#endif

#include "fsl_tpm_driver.h"

uint8_t tpm_instance = 1U;
uint8_t tpm_channel = 1U;

void initSG90();

void setSG90Position(uint32_t dutycycle);

void readSG90Registers();

void printTpmPwmParams(tpm_pwm_param_t * params);

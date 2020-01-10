#include "fsl_cmp_driver.h"
#include "fsl_cmp_hal.h"

void ddmain();
void configureCMP();

cmp_user_config_t cmp_cfg = {
	.hystersisMode = kCmpHystersisOfLevel0, 
	.pinoutEnable = true, 
	.pinoutUnfilteredEnable = true,
	.invertEnable = true,
	.highSpeedEnable = false,
	.risingIntEnable = false,
	.fallingIntEnable = false, 
	.plusChnMux = kCmpInputChn0, 
	.minusChnMux = kCmpInputChnDac,
	.triggerEnable = false,
};

cmp_dac_config_t cmp_dac_cfg = {
	.refVoltSrcMode = kCmpDacRefVoltSrcOf2,
	.dacValue = 0x05,
};

void ddStartCMP();
void ddStopCMP();

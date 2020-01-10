#include <stdio.h>
#include <stdlib.h>
#include <string.h>


#include "fsl_misc_utilities.h"
#include "fsl_device_registers.h"
#include "fsl_i2c_master_driver.h"
#include "fsl_rtc_driver.h"
#include "fsl_clock_manager.h"
#include "fsl_power_manager.h"
#include "fsl_mcglite_hal.h"
#include "fsl_port_hal.h"
#include "fsl_lpuart_driver.h"
#include "fsl_llwu_hal.h"
#include "fsl_pmc_hal.h"

#include "gpio_pins.h"
#include "SEGGER_RTT.h"
#include "warp.h"

#include "devMMA8451Q.h"
#include "devSG90.h"
#include "dropDetect.h"

typedef enum {
	init,
	to_standby,
	to_active,
	active,
	deployed,
} ddState_t;

/* 
char mode_names[6][6] =
{
	"int",
	"t",
	"tactv",
	"actv",
	"dplyd",
}; */


// State transisitions
//	init  ->  to_active  ->  active  ->  to_standby -> VLLS -> |
//	 |  <-----------------------------------------------------LLWU 


void 		ddSleep();
const uint16_t 	I2C_PULLUP_VALUE = 32768; 
bool 		TransientDebounceTracker();
void 		initLLWU();

bool tumbling;

ddState_t ddState = init;
void ddmain()
{
	do
	{
		///SEGGER_RTT_printf(0, "\r Mode: %s                        ", mode_names[ddState]);
		SEGGER_RTT_printf(0, "\r Mode: %d                        ", ddState);
		switch(ddState)
		{
			case init:

				// initialiseMMA8451Q	
				enableI2Cpins(I2C_PULLUP_VALUE);
				OSA_TimeDelay(1000);
				WarpStatus status; 
				status = configureSensorMMA8451QDropDetect(1800);
				if (status !=kWarpStatusOK) {SEGGER_RTT_WriteString(0, "\r\n\tconfigureSensorMMA8541QDropDetect failed");}
				status = configureSensorMMA8451QTransientDetect(1800);
				if (status !=kWarpStatusOK) {SEGGER_RTT_WriteString(0, "\r\n\tconfigureSensorMMA8541QTransientDetect failed");}


				// Initialise servo 
				initSG90();
				setSG90Position(15U);
				OSA_TimeDelay(1000); // Allow servo to move to initial position

				ddState = to_active;
				break;
			case to_active:
				if (MMA8451QTransientInterruptEnable(I2C_PULLUP_VALUE, false /* disable Interrupt */)){
					SEGGER_RTT_WriteString(0, "Disable interupt failed");
				}
				// turn on clocks
				// initialise servo
				initSG90();
				setSG90Position(15U);
				// turn on drop detection
				// turn on tumble detection
				ddState = active;
				break;
			case active:
				// check for tumble or drop
				tumbling = false;
				if (checkMMA8451QDropDetectEventLatch() | tumbling)
				{
				// 	spin servo
					setSG90Position(5U);
				 	ddState = deployed;
					break;
				}
				else if (TransientDebounceTracker())
				{
					ddState = to_standby;
				}
				break;
			case to_standby:
				// disable drop detection?
				// disable tumble detection?
				DeinitSG90(); // turn off servo and TPM to reduce power consumption
				// turn off clocks
				
				// Enable transient interrupt
				if (MMA8451QTransientInterruptEnable(I2C_PULLUP_VALUE, true /* enable Interrupt */)){
					SEGGER_RTT_WriteString(0, "Enable interupt failed");
				}
				// ddStartCMP();
				OSA_TimeDelay(300);
				// Clear transient interrupt register
				checkMMA8451QTransientDetectEventLatch(); // Clear latch

				ddSleep(); // go to VLLS0 with interrupt wakeup enabled sleep to reduce power consumption
				
				// THIS SHOULD NEVER BE REACHED AS THE MCU WILL BE IN VLLS, AND RESET THROUGH THE 
				// RESET VECTOR				
				break;
			case deployed:
				break;
		}
	}while(ddState!=deployed);
	ddState = init;
}

void ddSleep(){
	initLLWU();
	OSA_TimeDelay(300);
	warpSetLowPowerMode(kWarpPowerModeVLLS1, 0);
	SEGGER_RTT_WriteString(0, "Failed to set power mode to LLWU");
}

void initLLWU()
{	
	SEGGER_RTT_WriteString(0, "\r\n Changing pin A12 to CMP0_IN0");
	PORT_HAL_SetMuxMode(PORTA_BASE, 12, 0U); // Set to CMP in
	PORT_HAL_SetMuxMode(PORTB_BASE, 0, kPortMuxAsGpio); // Set to LLWU_P4, i.e. Alt 1
	//GPIO_DRV_InputPinInit( );
	//llwu_external_pin_modes_t 
	//llwu_wakeup_pin_t 
	LLWU_HAL_SetExternalInputPinMode(LLWU_BASE, kLlwuExternalPinChangeDetect, kLlwuWakeupPin4);
	PMC_HAL_SetClearAckIsolation(PMC_BASE);
}

/* Debounce counter for transition from active to standby */
uint16_t transient_debounce_counter = 0U;
const uint16_t TRANSIENT_DEBOUNCE_THRESHOLD = 1000U;
// Used to check if drone is inactive to move to standby
bool TransientDebounceTracker()
{	
	if (!checkMMA8451QTransientDetectEventLatch())
	{
		// if stationary increment counter and check against threshold
		if (++transient_debounce_counter >  TRANSIENT_DEBOUNCE_THRESHOLD)
		{
			// drone is stationary
			transient_debounce_counter = 0U; // reset
			return true; // drone is stationary
		}
	}
	else
	{
		// else reset counter to zero
		transient_debounce_counter = 0U;
	}
	return false;
}

uint32_t cmp_instance;
cmp_state_t cmp_state;

void ddStartCMP()
{
	SEGGER_RTT_WriteString(0, "\r\n Configuring CMP");
	SEGGER_RTT_WriteString(0, "\r\n Changing pin A12 to CMP0_IN0");
	PORT_HAL_SetMuxMode(PORTA_BASE, 12, 0U); // Set to CMP in
	SEGGER_RTT_WriteString(0, "\r\n Changing pin A2 to CMP0_OUT");
	PORT_HAL_SetMuxMode(PORTA_BASE, 2, kPortMuxAlt2); // Set to CMP out 
	SEGGER_RTT_WriteString(0, "\r\n Enabling CMP");
	CMP_DRV_Init(cmp_instance, &cmp_cfg, &cmp_state);
	SEGGER_RTT_WriteString(0, "\r\n Enabling DAC");
	CMP_DRV_EnableDac(cmp_instance, &cmp_dac_cfg);
	SEGGER_RTT_WriteString(0, "\r\n Starting CMP");
	CMP_DRV_Start(cmp_instance);
	SEGGER_RTT_WriteString(0, "\r\n Done CMP");
}

void ddStopCMP()
{
	CMP_DRV_Stop(cmp_instance);
	CMP_DRV_Deinit(cmp_instance);
	CMP_DRV_DisableDac(cmp_instance);
}

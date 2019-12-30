/*
	Authored 2016-2018. Phillip Stanley-Marbell, Youchao Wang.

	All rights reserved.

	Redistribution and use in source and binary forms, with or without
	modification, are permitted provided that the following conditions
	are met:

	*	Redistributions of source code must retain the above
		copyright notice, this list of conditions and the following
		disclaimer.

	*	Redistributions in binary form must reproduce the above
		copyright notice, this list of conditions and the following
		disclaimer in the documentation and/or other materials
		provided with the distribution.

	*	Neither the name of the author nor the names of its
		contributors may be used to endorse or promote products
		derived from this software without specific prior written
		permission.

	THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
	"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
	LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
	FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
	COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
	INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
	BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
	LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
	CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
	LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
	ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
	POSSIBILITY OF SUCH DAMAGE.
*/
#include <stdlib.h>

#include "fsl_misc_utilities.h"
#include "fsl_device_registers.h"
#include "fsl_i2c_master_driver.h"
#include "fsl_spi_master_driver.h"
#include "fsl_rtc_driver.h"
#include "fsl_clock_manager.h"
#include "fsl_power_manager.h"
#include "fsl_mcglite_hal.h"
#include "fsl_port_hal.h"

#include "gpio_pins.h"
#include "SEGGER_RTT.h"
#include "warp.h"

#include "devMMA8451Q.h"

extern volatile WarpI2CDeviceState	deviceMMA8451QState;
extern volatile uint32_t		gWarpI2cBaudRateKbps;
extern volatile uint32_t		gWarpI2cTimeoutMilliseconds;
extern volatile uint32_t		gWarpSupplySettlingDelayMilliseconds;


void
initMMA8451Q(const uint8_t i2cAddress, WarpI2CDeviceState volatile *  deviceStatePointer)
{
	deviceStatePointer->i2cAddress	= i2cAddress;
	deviceStatePointer->signalType	= (	kWarpTypeMaskAccelerationX |
						kWarpTypeMaskAccelerationY |
						kWarpTypeMaskAccelerationZ |
						kWarpTypeMaskTemperature
					);
	return;
}

WarpStatus
writeSensorRegisterMMA8451Q(uint8_t deviceRegister, uint8_t payload, uint16_t menuI2cPullupValue)
{
	uint8_t		payloadByte[1], commandByte[1];
	i2c_status_t	status;

	switch (deviceRegister)
	{
		case 0x09: case 0x0a: case 0x0e: case 0x0f:
		case 0x11: case 0x12: case 0x13: case 0x14:
		case 0x15: case 0x17: case 0x18: case 0x1d:
		case 0x1f: case 0x20: case 0x21: case 0x23:
		case 0x24: case 0x25: case 0x26: case 0x27:
		case 0x28: case 0x29: case 0x2a: case 0x2b:
		case 0x2c: case 0x2d: case 0x2e: case 0x2f:
		case 0x30: case 0x31:
		{
			/* OK */
			break;
		}
		
		default:
		{
			return kWarpStatusBadDeviceCommand;
		}
	}

	i2c_device_t slave =
	{
		.address = deviceMMA8451QState.i2cAddress,
		.baudRate_kbps = gWarpI2cBaudRateKbps
	};

	commandByte[0] = deviceRegister;
	payloadByte[0] = payload;
	status = I2C_DRV_MasterSendDataBlocking(
							0 /* I2C instance */,
							&slave,
							commandByte,
							1,
							payloadByte,
							1,
							gWarpI2cTimeoutMilliseconds);
	if (status != kStatus_I2C_Success)
	{
		SEGGER_RTT_printf(0, "\r\n\t writeSensorRegisterMMA8451Q failed, code %d", status);
		return kWarpStatusDeviceCommunicationFailed;
	}

	return kWarpStatusOK;
}

WarpStatus
configureSensorMMA8451Q(uint8_t payloadF_SETUP, uint8_t payloadCTRL_REG1, uint16_t menuI2cPullupValue)
{
	WarpStatus	i2cWriteStatus1, i2cWriteStatus2;

	i2cWriteStatus1 = writeSensorRegisterMMA8451Q(kWarpSensorConfigurationRegisterMMA8451QF_SETUP /* register address F_SETUP */,
							payloadF_SETUP /* payload: Disable FIFO */,
							menuI2cPullupValue);

	i2cWriteStatus2 = writeSensorRegisterMMA8451Q(kWarpSensorConfigurationRegisterMMA8451QCTRL_REG1 /* register address CTRL_REG1 */,
							payloadCTRL_REG1 /* payload */,
							menuI2cPullupValue);

	if (i2cWriteStatus1 | i2cWriteStatus2)
	{
		SEGGER_RTT_printf(0, "\r\n\t configureSensorMMA8451Q failed, codes %d, %d", i2cWriteStatus1, i2cWriteStatus2);
	}
	return (i2cWriteStatus1 | i2cWriteStatus2);
}


WarpStatus
configureSensorMMA8451QDropDetect(uint16_t menuI2cPullupValue)
{
	configureSensorMMA8451Q(0x00, 0x00, menuI2cPullupValue);
	/* Run AFTER running  configureSensorMMA8451Q*/

	WarpStatus	i2cWriteStatus1, i2cWriteStatus2, i2cWriteStatus3;

	i2cWriteStatus1 = writeSensorRegisterMMA8451Q(0x15 /* FF_MT_CFG register address*/,
							0xB8 /* payload: Event latch enabled, freefall detect on all axes  */,
							menuI2cPullupValue);

	i2cWriteStatus2 = writeSensorRegisterMMA8451Q(0x17 /* register address FF_MT_THS */,
							0x04 /* payload 0.25g */,
							menuI2cPullupValue);

	i2cWriteStatus3 = writeSensorRegisterMMA8451Q(0x18 /* register address Debounce count register address*/,
							0x20 /* payload */,
							menuI2cPullupValue);

	configureSensorMMA8451Q(0x00, 0x01 /* set active */, menuI2cPullupValue);
	return (i2cWriteStatus1 | i2cWriteStatus2 | i2cWriteStatus3);
}

bool
checkMMA8451QDropDetectEventLatch()
{	
	WarpStatus status = readSensorRegisterMMA8451Q(0x16 /* register address */, 1 /* number of bytes */);
	if (status == kWarpStatusOK){
		uint8_t regValue = deviceMMA8451QState.i2cBuffer[0];
		return (bool) ((regValue>>7) & 0x01);
	}
	else{
		SEGGER_RTT_WriteString(0, "Read Drop detect register Failed");
		return false;
	}
}

bool
checkMMA8451QTransientDetectEventLatch()
{	
	WarpStatus status = readSensorRegisterMMA8451Q(0x1E /* SRC register address */, 1 /* number of bytes */);
	if (status == kWarpStatusOK){
		uint8_t regValue = deviceMMA8451QState.i2cBuffer[0];
		SEGGER_RTT_printf(0, "\r\n regValue %02x \n", regValue);
		return (bool) ((regValue>>6) & 0x01);
	}
	else{
		SEGGER_RTT_WriteString(0, "Read Transient detect register Failed");
		return false;
	}
}

void
checkMMA8451QInterruptStatus()
{	
	WarpStatus status = readSensorRegisterMMA8451Q(0x0c /* INT_SOURCE register */, 1 /* number of bytes */);
	if (status == kWarpStatusOK){
		uint8_t regValue = deviceMMA8451QState.i2cBuffer[0];
		SEGGER_RTT_printf(0, "\r\n Interrupt source register %02x \n", regValue);
	}
	else{
		SEGGER_RTT_WriteString(0, "Read Interrupt Source register Failed");
	}
}

WarpStatus
configureSensorMMA8451QTransientDetect(uint16_t menuI2cPullupValue)
{
	configureSensorMMA8451Q(0x00, 0x00, menuI2cPullupValue);
	/* Run AFTER running
	 * configureSensorMMA8451Q(0x00, 0x00, menuI2cPullupValue);
	 */

	WarpStatus	i2cWriteStatus1, i2cWriteStatus2, i2cWriteStatus3;

	i2cWriteStatus1 = writeSensorRegisterMMA8451Q(0x1D /* TRANSIENT_CFG register address*/,
							0x1E /* payload: Event latch enabled, transient detect on all axes  */,
							menuI2cPullupValue);

	i2cWriteStatus2 = writeSensorRegisterMMA8451Q(0x1F /* register address TRANSIENT_THS */,
							0x05 /* payload 0.375g */,
							menuI2cPullupValue);

	i2cWriteStatus3 = writeSensorRegisterMMA8451Q(0x20 /* Debounce count register address*/,
							0x04 /* payload */,
							menuI2cPullupValue);

	configureSensorMMA8451Q(0x00, 0x01 /* set active */, menuI2cPullupValue);
	return (i2cWriteStatus1 | i2cWriteStatus2 | i2cWriteStatus3);
}

WarpStatus
MMA8451QTransientInterruptEnable(uint16_t menuI2cPullupValue, bool enableInterrupt)
{
	configureSensorMMA8451Q(0x00, 0x00 /* set inactive */, menuI2cPullupValue);

	WarpStatus	i2cWriteStatus1, i2cWriteStatus2, i2cWriteStatus3;


	i2cWriteStatus1 = writeSensorRegisterMMA8451Q(0x2C /* CTRL_REG3 register address*/,
							0x20, // Active high interrupts
							menuI2cPullupValue);

	uint8_t payload = enableInterrupt? 0x20: 0x00;
	i2cWriteStatus2 = writeSensorRegisterMMA8451Q(0x2D /* CTRL_REG4 register address*/,
							payload,
							menuI2cPullupValue);

	i2cWriteStatus3 = writeSensorRegisterMMA8451Q(0x2E /* CTRL_REG5 aim at INT2 pin */,
							0x00 /* aim all at INT2 pin */,
							menuI2cPullupValue);

	PORT_HAL_SetMuxMode(PORTA_BASE, 12, kPortMuxAsGpio); // Set to PTA12, i.e. Alt 1

	configureSensorMMA8451Q(0x00, 0x01 /* set active */, menuI2cPullupValue);
	return (i2cWriteStatus1 | i2cWriteStatus2 | i2cWriteStatus3 );
}


WarpStatus
readSensorRegisterMMA8451Q(uint8_t deviceRegister, int numberOfBytes)
{
	uint8_t		cmdBuf[1] = {0xFF};
	i2c_status_t	status;


	USED(numberOfBytes);
	switch (deviceRegister)
	{
		case 0x00: case 0x01: case 0x02: case 0x03: 
		case 0x04: case 0x05: case 0x06: case 0x09:
		case 0x0a: case 0x0b: case 0x0c: case 0x0d:
		case 0x0e: case 0x0f: case 0x10: case 0x11:
		case 0x12: case 0x13: case 0x14: case 0x15:
		case 0x16: case 0x17: case 0x18: case 0x1d:
		case 0x1e: case 0x1f: case 0x20: case 0x21:
		case 0x22: case 0x23: case 0x24: case 0x25:
		case 0x26: case 0x27: case 0x28: case 0x29:
		case 0x2a: case 0x2b: case 0x2c: case 0x2d:
		case 0x2e: case 0x2f: case 0x30: case 0x31:
		{
			/* OK */
			break;
		}
		
		default:
		{
			return kWarpStatusBadDeviceCommand;
		}
	}


	i2c_device_t slave =
	{
		.address = deviceMMA8451QState.i2cAddress,
		.baudRate_kbps = gWarpI2cBaudRateKbps
	};


	cmdBuf[0] = deviceRegister;

	status = I2C_DRV_MasterReceiveDataBlocking(
							0 /* I2C peripheral instance */,
							&slave,
							cmdBuf,
							1,
							(uint8_t *)deviceMMA8451QState.i2cBuffer,
							numberOfBytes,
							gWarpI2cTimeoutMilliseconds);

	if (status != kStatus_I2C_Success)
	{
		return kWarpStatusDeviceCommunicationFailed;
	}

	return kWarpStatusOK;
}

void
printSensorDataMMA8451Q(bool hexModeFlag)
{
	uint16_t	readSensorRegisterValueLSB;
	uint16_t	readSensorRegisterValueMSB;
	int16_t		readSensorRegisterValueCombined;
	WarpStatus	i2cReadStatus;


	/*
	 *	From the MMA8451Q datasheet:
	 *
	 *		"A random read access to the LSB registers is not possible.
	 *		Reading the MSB register and then the LSB register in sequence
	 *		ensures that both bytes (LSB and MSB) belong to the same data
	 *		sample, even if a new data sample arrives between reading the
	 *		MSB and the LSB byte."
	 *
	 *	We therefore do 2-byte read transactions, for each of the registers.
	 *	We could also improve things by doing a 6-byte read transaction.
	 */
	i2cReadStatus = readSensorRegisterMMA8451Q(kWarpSensorOutputRegisterMMA8451QOUT_X_MSB, 2 /* numberOfBytes */);
	readSensorRegisterValueMSB = deviceMMA8451QState.i2cBuffer[0];
	readSensorRegisterValueLSB = deviceMMA8451QState.i2cBuffer[1];
	readSensorRegisterValueCombined = ((readSensorRegisterValueMSB & 0xFF) << 6) | (readSensorRegisterValueLSB >> 2);

	/*
	 *	Sign extend the 14-bit value based on knowledge that upper 2 bit are 0:
	 */
	readSensorRegisterValueCombined = (readSensorRegisterValueCombined ^ (1 << 13)) - (1 << 13);


	if (i2cReadStatus != kWarpStatusOK)
	{
		SEGGER_RTT_WriteString(0, " ----,");
	}
	else
	{
		if (hexModeFlag)
		{
			SEGGER_RTT_printf(0, " 0x%02x 0x%02x,", readSensorRegisterValueMSB, readSensorRegisterValueLSB);
		}
		else
		{
			SEGGER_RTT_printf(0, " %d,", readSensorRegisterValueCombined);
		}
	}


	i2cReadStatus = readSensorRegisterMMA8451Q(kWarpSensorOutputRegisterMMA8451QOUT_Y_MSB, 2 /* numberOfBytes */);
	readSensorRegisterValueMSB = deviceMMA8451QState.i2cBuffer[0];
	readSensorRegisterValueLSB = deviceMMA8451QState.i2cBuffer[1];
	readSensorRegisterValueCombined = ((readSensorRegisterValueMSB & 0xFF) << 6) | (readSensorRegisterValueLSB >> 2);

	/*
	 *	Sign extend the 14-bit value based on knowledge that upper 2 bit are 0:
	 */
	readSensorRegisterValueCombined = (readSensorRegisterValueCombined ^ (1 << 13)) - (1 << 13);


	if (i2cReadStatus != kWarpStatusOK)
	{
		SEGGER_RTT_WriteString(0, " ----,");
	}
	else
	{
		if (hexModeFlag)
		{
			SEGGER_RTT_printf(0, " 0x%02x 0x%02x,", readSensorRegisterValueMSB, readSensorRegisterValueLSB);
		}
		else
		{
			SEGGER_RTT_printf(0, " %d,", readSensorRegisterValueCombined);
		}
	}


	i2cReadStatus = readSensorRegisterMMA8451Q(kWarpSensorOutputRegisterMMA8451QOUT_Z_MSB, 2 /* numberOfBytes */);
	readSensorRegisterValueMSB = deviceMMA8451QState.i2cBuffer[0];
	readSensorRegisterValueLSB = deviceMMA8451QState.i2cBuffer[1];
	readSensorRegisterValueCombined = ((readSensorRegisterValueMSB & 0xFF) << 6) | (readSensorRegisterValueLSB >> 2);

	/*
	 *	Sign extend the 14-bit value based on knowledge that upper 2 bit are 0:
	 */
	readSensorRegisterValueCombined = (readSensorRegisterValueCombined ^ (1 << 13)) - (1 << 13);


	if (i2cReadStatus != kWarpStatusOK)
	{
		SEGGER_RTT_WriteString(0, " ----,");
	}
	else
	{
		if (hexModeFlag)
		{
			SEGGER_RTT_printf(0, " 0x%02x 0x%02x,", readSensorRegisterValueMSB, readSensorRegisterValueLSB);
		}
		else
		{
			SEGGER_RTT_printf(0, " %d,", readSensorRegisterValueCombined);
		}
	}
}

//*****************************************************************************
//
// pressure_AD7294.c - Example to use of the SensorLib with the AD7294
//
// Copyright (c) 2013 Texas Instruments Incorporated.  All rights reserved.
// Software License Agreement
// 
// Texas Instruments (TI) is supplying this software for use solely and
// exclusively on TI's microcontroller products. The software is owned by
// TI and/or its suppliers, and is protected under applicable copyright
// laws. You may not combine this software with "viral" open-source
// software in order to form a larger program.
// 
// THIS SOFTWARE IS PROVIDED "AS IS" AND WITH ALL FAULTS.
// NO WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT
// NOT LIMITED TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. TI SHALL NOT, UNDER ANY
// CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL, OR CONSEQUENTIAL
// DAMAGES, FOR ANY REASON WHATSOEVER.
// 
// This is part of revision 1.0 of the EK-TM4C123GXL Firmware Package.
//
//*****************************************************************************
/*
#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_ints.h"
#include "driverlib/debug.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/sysctl.h"
#include "driverlib/systick.h"
#include "driverlib/uart.h"
#include "utils/uartstdio.h"
#include "sensorlib/hw_AD7294.h"
#include "sensorlib/i2cm_drv.h"
#include "sensorlib/AD7294.h"
#include "drivers/rgb.h"

#include <stdarg.h>
#include "inc/hw_i2c.h"
#include "inc/hw_gpio.h"
#include "driverlib/i2c.h"

*/

#include "stdint.h"
#include "stdbool.h"
#include <stdarg.h>
#include <math.h>
#include "time.h"
#include "touch.h"
#include "images.h"

#include "inc/hw_memmap.h"
#include "inc/hw_nvic.h"
#include "inc/hw_sysctl.h"
#include "inc/hw_types.h"
#include "inc/hw_ints.h"
#include "inc/hw_i2c.h"
#include "inc/hw_gpio.h"

#include "driverlib/fpu.h"
#include "driverlib/gpio.h"
#include "driverlib/flash.h"
#include "driverlib/sysctl.h"
#include "driverlib/systick.h"
#include "driverlib/uart.h"
#include "driverlib/udma.h"
#include "driverlib/rom.h"
#include "driverlib/debug.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/i2c.h"

#include "grlib/grlib.h"
#include "grlib/widget.h"
#include "grlib/canvas.h"
#include "grlib/checkbox.h"
#include "grlib/container.h"
#include "grlib/pushbutton.h"
#include "grlib/radiobutton.h"
#include "grlib/slider.h"

#include "utils/ustdlib.h"
#include "utils/uartstdio.h"

#include "Kentec320x240x16_ssd2119_8bit.h"

#include "sensorlib/hw_AD7294.h"
#include "sensorlib/AD7294.h"
//#include "sensorlib/i2cm_drv.h"

#include "drivers/rgb.h"

#define	timeout 4e5

#define P11_x (GrContextDpyWidthGet(&sContext) / 10)+40
#define P21_x (GrContextDpyWidthGet(&sContext) / 10)+40
#define P31_x (GrContextDpyWidthGet(&sContext) / 10)+40
#define P41_x (GrContextDpyWidthGet(&sContext) / 10)+40
#define P51_x (GrContextDpyWidthGet(&sContext) / 10)+40
#define P61_x (GrContextDpyWidthGet(&sContext) / 10)+40

#define P12_x (GrContextDpyWidthGet(&sContext) / 2)+80
#define P22_x (GrContextDpyWidthGet(&sContext) / 2)+80
#define P32_x (GrContextDpyWidthGet(&sContext) / 2)+80
#define P42_x (GrContextDpyWidthGet(&sContext) / 2)+80
#define P52_x (GrContextDpyWidthGet(&sContext) / 2)+80
#define P62_x (GrContextDpyWidthGet(&sContext) / 2)+80

#define P11_y 50
#define P21_y 80
#define P31_y 110
#define P41_y 140
#define P51_y 170
#define P61_y 200

#define P12_y 50
#define P22_y 80
#define P32_y 110
#define P42_y 140
#define P52_y 170
#define P62_y 200






//*****************************************************************************
// Define global variables

    float fTemp1=0, fTemp2=0, fTempInt=0;
    float fVin0=0, fVin1=0, fVin2=0, fVin3=0;
    float fIsense1=0, fIsense2=0;
    float fBTemp1[3] = {0,0,0}, fBTemp2[3] = {0,0,0}, fBTempInt[3] = {0,0,0};
    float fBVin0[3] = {0,0,0}, fBVin1[3] = {0,0,0}, fBVin2[3] = {0,0,0}, fBVin3[3] = {0,0,0};
    float fBIsense1[3] = {0,0,0}, fBIsense2[3] = {0,0,0};
    float fADC_Val=0;
    float fRsense = 0.010;
    float fVref0 = 2.5, fVref1 = 2.5, fVref2 = 2.5, fVref3 = 2.5;

    uint16_t i16average = 16;
    uint8_t	SysTick_Semafore = 1;


//*****************************************************************************
//
// Define AD7294 I2C Address.
//
//*****************************************************************************
#define AD7294_I2C_ADDRESS  0x61

//*****************************************************************************
//
//! \addtogroup example_list
//! <h1>pressure Measurement with the AD7294 (pressure_AD7294)</h1>
//!
//! This example demonstrates the basic use of the Sensor Library, TM4C123G
//! LaunchPad and the SensHub BoosterPack to obtain air pressure and
//! temperature measurements with the AD7294 sensor.
//!
//! Connect a serial terminal program to the LaunchPad's ICDI virtual serial
//! port at 115,200 baud.  Use eight bits per byte, no parity and one stop bit.
//! The raw sensor measurements are printed to the terminal.  The RGB LED
//! blinks at 1Hz once the initialization is complete and the example is
//! running.
//
//*****************************************************************************




//*****************************************************************************
//
// The DMA control structure table.
//
//*****************************************************************************

    #ifdef ewarm
    #pragma data_alignment=1024
    tDMAControlTable sDMAControlTable[64];
    #elif defined(ccs)
    #pragma DATA_ALIGN(sDMAControlTable, 1024)
    tDMAControlTable sDMAControlTable[64];
    #else
    tDMAControlTable sDMAControlTable[64] __attribute__ ((aligned(1024)));
    #endif



//*****************************************************************************
//
// Global array for holding the color values for the RGB.
//
//*****************************************************************************
uint32_t g_pui32Colors[3];

//*****************************************************************************
//
// Global instance structure for the I2C master driver.
//
//*****************************************************************************
// tI2CMInstance g_sI2CInst;

//*****************************************************************************
//
// Global instance structure for the AD7294 sensor driver.
//
//*****************************************************************************
//tAD7294 g_sAD7294Inst;

//*****************************************************************************
//
// Global new data flag to alert main that AD7294 data is ready.
//
//*****************************************************************************
 //volatile uint_fast8_t g_vui8DataFlag;

//*****************************************************************************
//
// The error routine that is called if the driver library encounters an error.
//
//*****************************************************************************
#ifdef DEBUG
void
__error__(char *pcFilename, uint32_t ui32Line)
{
}
#endif
//*****************************************************************************
// Check if I2C is available. - Wait until MCU is done transferring.
uint8_t I2CAvailable()
{
bool 				test_val=true;
unsigned long    	k=0;

test_val=true;
k = 0;
while(test_val&(k<timeout))
	{
	test_val=I2CMasterBusy(I2C3_BASE);
	k = k+1;
	}
if (k>=timeout)
	{
	return 1;
	}
return 0;

}
//*****************************************************************************
//sends an I2C command to the specified slave
uint8_t I2CSend(uint8_t slave_addr, uint8_t num_of_args, ...)
{
	//bool 				test_val=true;
	uint8_t 			i=0;
	uint8_t 			status=0;

    // Tell the master module what address it will place on the bus when
    // communicating with the slave.
    I2CMasterSlaveAddrSet(I2C3_BASE, slave_addr, false);

    //stores list of variable number of arguments
    va_list vargs;

    //specifies the va_list to "open" and the last fixed argument
    //so vargs knows where to start looking
    va_start(vargs, num_of_args);

    //put data to be sent into FIFO
    I2CMasterDataPut(I2C3_BASE, va_arg(vargs, uint32_t));

    //if there is only one argument, we only need to use the
    //single send I2C function
    if(num_of_args == 1)
    {
        //Initiate send of data from the MCU
        I2CMasterControl(I2C3_BASE, I2C_MASTER_CMD_SINGLE_SEND);

        // Wait until MCU is done transferring.
        status = I2CAvailable();
        if (status)
        	{
        	return status;
        	}

        //"close" variable argument list
        va_end(vargs);
    }

    //otherwise, we start transmission of multiple bytes on the
    //I2C bus
    else
    {
        //Initiate send of data from the MCU
        I2CMasterControl(I2C3_BASE, I2C_MASTER_CMD_BURST_SEND_START);

        // Wait until MCU is done transferring.
        status = I2CAvailable();
        if (status)
        	{
        	return status;
        	}

        //send num_of_args-2 pieces of data, using the
        //BURST_SEND_CONT command of the I2C module
        for(i = 1; i < (num_of_args - 1); i++)
        {
            //put next piece of data into I2C FIFO
            I2CMasterDataPut(I2C3_BASE, va_arg(vargs, uint32_t));
            //send next data that was just placed into FIFO
            I2CMasterControl(I2C3_BASE, I2C_MASTER_CMD_BURST_SEND_CONT);

            // Wait until MCU is done transferring.
            status = I2CAvailable();
            if (status)
            	{
            	return status;
            	}
        }

        //put last piece of data into I2C FIFO
        I2CMasterDataPut(I2C3_BASE, va_arg(vargs, uint32_t));
        //send next data that was just placed into FIFO
        I2CMasterControl(I2C3_BASE, I2C_MASTER_CMD_BURST_SEND_FINISH);
        // Wait until MCU is done transferring.
        status = I2CAvailable();
        if (status)
        	{
        	return status;
        	}

        //"close" variable args list
        va_end(vargs);
    }

    return 0;
}
//*****************************************************************************
//read single word from specified register on slave device
uint32_t I2CReceive(uint32_t slave_addr, uint8_t reg)
{
	uint8_t	status = 0;
    //specify that we are writing (a register address) to the
    //slave device
    I2CMasterSlaveAddrSet(I2C3_BASE, slave_addr, false);

    //specify register to be read
    I2CMasterDataPut(I2C3_BASE, reg);

    //send control byte and register address byte to slave device
    I2CMasterControl(I2C3_BASE, I2C_MASTER_CMD_BURST_SEND_START);

    //wait for MCU to finish transaction
    //while(I2CMasterBusy(I2C3_BASE));
    status = I2CAvailable();
    if (status)
    	{
    	return status;
    	}

    //specify that we are going to read from slave device
    I2CMasterSlaveAddrSet(I2C3_BASE, slave_addr, true);

    //send control byte and read from the register we
    //specified
    I2CMasterControl(I2C3_BASE, I2C_MASTER_CMD_SINGLE_RECEIVE);

    //wait for MCU to finish transaction
    //while(I2CMasterBusy(I2C3_BASE));
    status = I2CAvailable();
    if (status)
    	{
    	return status;
    	}

    //return data pulled from the specified register
    return I2CMasterDataGet(I2C3_BASE);
}
//*****************************************************************************
//read 2 bytes from specified register on slave device
uint8_t I2CReceive2(uint16_t *Register_Value, uint32_t slave_addr, uint8_t reg)
{
	uint8_t	status = 0;
    uint16_t Ret_Value_MSB=0;
    uint16_t Ret_Value_LSB=0;
    uint16_t Ret_Value=0;
    //specify that we are writing (a register address) to the
    //slave device
    I2CMasterSlaveAddrSet(I2C3_BASE, slave_addr, false);

    //specify register to be read
    I2CMasterDataPut(I2C3_BASE, reg);

    //send control byte and register address byte to slave device
    I2CMasterControl(I2C3_BASE, I2C_MASTER_CMD_BURST_SEND_START);

    //wait for MCU to finish transaction
    //while(I2CMasterBusy(I2C3_BASE));
    status = I2CAvailable();
    if (status)
    	{
    	return status;
    	}
    //specify that we are going to read from slave device
    I2CMasterSlaveAddrSet(I2C3_BASE, slave_addr, true);

    //Read first byte
    I2CMasterControl(I2C3_BASE, I2C_MASTER_CMD_BURST_RECEIVE_START);
    //wait for MCU to finish transaction
    //while(I2CMasterBusy(I2C3_BASE));
    status = I2CAvailable();
    if (status)
    	{
    	return status;
    	}
    Ret_Value_MSB = I2CMasterDataGet(I2C3_BASE);
    //Read second byte
    I2CMasterControl(I2C3_BASE, I2C_MASTER_CMD_BURST_RECEIVE_FINISH);
    //wait for MCU to finish transaction
    //while(I2CMasterBusy(I2C3_BASE));
    status = I2CAvailable();
    if (status)
    	{
    	return status;
    	}
    Ret_Value_LSB=I2CMasterDataGet(I2C3_BASE);
    Ret_Value = (Ret_Value_MSB<<8);
    Ret_Value = Ret_Value+Ret_Value_LSB;
    *Register_Value = Ret_Value;
    return status;

}
//*****************************************************************************
void	Median_Filter_for_3(float *data_in)
{
	uint8_t	i=0;
	float	data_sort[3];
	float	x=0;
	for (i=0;i<3;i++) data_sort[i]=data_in[i];
	//Bubble sort the data by size :
	if (data_sort[2]<data_sort[1])
	{
		x=data_sort[2];
		data_sort[2] = data_sort[1];
		data_sort[1] = x;
	}

	if (data_sort[1]<data_sort[0])
	{
		x=data_sort[1];
		data_sort[1] = data_sort[0];
		data_sort[0] = x;
	}
	if (data_sort[2]<data_sort[1])
	{
		x=data_sort[2];
		data_sort[2] = data_sort[1];
		data_sort[1] = x;
	}


	data_in[1] = data_sort[1];	//Median filtering on center value

}

//*****************************************************************************
//  Read Results from AD7294

uint8_t AD7294_Read_Results(void)
{

	    uint16_t i16Result_Reg = 0;
	    uint16_t i16Channel_Number=0;
	    uint16_t i16Tsense1_Reg=0;
	    uint16_t i16Tsense2_Reg=0;
	    uint16_t i16TsenseInt_Reg=0;
	    uint8_t status = 0;

	// Get Voltages (and Currents) from Result RegfVoltagefVoltagester:
	status = I2CReceive2(&i16Result_Reg, AD7294_I2C_ADDRESS ,AD7294_R_Result);
	if (status)
		{
		return status;
		}
	i16Channel_Number = (i16Result_Reg&AD7294_Channel_Mask)>>12;

	if (i16Channel_Number<4)		// Voltage read Vin0 to VIn3
	{
		fADC_Val = i16Result_Reg&AD7294_B11_to_B0_Mask;
	}
	else							// Current read Isense1 to Isense2
	{
		fADC_Val = (i16Result_Reg&AD7294_B10_to_B0_Mask)-(i16Result_Reg&AD7294_B11_Bit);
	}

	switch(i16Channel_Number)
	{
	    case 0  :		//Vin0
        	fADC_Val = fADC_Val*fVref0/4096;
        	fBVin0[0] = fBVin0[1];		// Load New Value into buffer
        	fBVin0[1] = fBVin0[2];
        	fBVin0[2] = fADC_Val;
	    	fVin0 = (fVin0*(i16average-1)+fBVin0[0])/i16average;
	    	Median_Filter_for_3(&fBVin0[0]);
	       break;
	    case 1  :		//Vin1
        	fADC_Val = fADC_Val*fVref1/4096;
        	fBVin1[0] = fBVin1[1];		// Load New Value into buffer
        	fBVin1[1] = fBVin1[2];
        	fBVin1[2] = fADC_Val;
	    	fVin1 = (fVin1*(i16average-1)+fBVin1[0])/i16average;
	    	Median_Filter_for_3(&fBVin1[0]);
	       break;
	    case 2  :		//Vin2
        	fADC_Val = fADC_Val*fVref2/4096;
        	fBVin2[0] = fBVin2[1];		// Load New Value into buffer
        	fBVin2[1] = fBVin2[2];
        	fBVin2[2] = fADC_Val;
	    	fVin2 = (fVin2*(i16average-1)+fBVin2[0])/i16average;
	    	Median_Filter_for_3(&fBVin2[0]);
	       break;
	    case 3  :		//Vin3
        	fADC_Val = fADC_Val*fVref3/4096;
        	fBVin3[0] = fBVin3[1];		// Load New Value into buffer
        	fBVin3[1] = fBVin3[2];
        	fBVin3[2] = fADC_Val;
	    	fVin3 = (fVin3*(i16average-1)+fBVin3[0])/i16average;
	    	Median_Filter_for_3(&fBVin3[0]);
	       break;
	    case 4  :		//Isense1
	    	fADC_Val = fADC_Val*0.4/4096;
	    	fBIsense1[0] = fBIsense1[1];		// Load New Value into buffer
        	fBIsense1[1] = fBIsense1[2];
        	fBIsense1[2] = fADC_Val;
	    	fIsense1 = (fIsense1*(i16average-1)+(fBIsense1[0]/fRsense))/i16average;
	    	Median_Filter_for_3(&fBIsense1[0]);
	       break;
	    case 5  :		//I2ense2
	    	fADC_Val = fADC_Val*0.4/4096;
	    	fBIsense2[0] = fBIsense2[1];		// Load New Value into buffer
        	fBIsense2[1] = fBIsense2[2];
        	fBIsense2[2] = fADC_Val;
	    	fIsense2 = (fIsense2*(i16average-1)+(fBIsense2[0]/fRsense))/i16average;
	    	Median_Filter_for_3(&fBIsense2[0]);
	       break;
	}


	// Get Temperatures:
	status = I2CReceive2(&i16Tsense1_Reg, AD7294_I2C_ADDRESS ,AD7294_R_Tsense_1);
	if (status)
		{
		return status;
		}
	i16Channel_Number = (i16Tsense1_Reg&AD7294_Channel_Mask)>>12;
	fADC_Val = ((i16Tsense1_Reg&AD7294_D9_to_D0_Mask)-(i16Tsense1_Reg&AD7294_D10_Bit))/4.;
	fBTemp1[0] = fBTemp1[1];		// Load New Value into buffer
	fBTemp1[1] = fBTemp1[2];
	fBTemp1[2] = fADC_Val;
	fTemp1 = (fTemp1*(i16average-1)+fBTemp1[0])/i16average;
	Median_Filter_for_3(&fBTemp1[0]);

	status = I2CReceive2(&i16Tsense2_Reg, AD7294_I2C_ADDRESS ,AD7294_R_Tsense_2);
	if (status)
		{
		return status;
		}
	i16Channel_Number = (i16Tsense2_Reg&AD7294_Channel_Mask)>>12;
	fADC_Val = ((i16Tsense2_Reg&AD7294_D9_to_D0_Mask)-(i16Tsense2_Reg&AD7294_D10_Bit))/4.;
	fBTemp2[0] = fBTemp2[1];		// Load New Value into buffer
	fBTemp2[1] = fBTemp2[2];
	fBTemp2[2] = fADC_Val;
	fTemp2 = (fTemp2*(i16average-1)+fBTemp2[0])/i16average;
	Median_Filter_for_3(&fBTemp2[0]);


	status = I2CReceive2(&i16TsenseInt_Reg, AD7294_I2C_ADDRESS ,AD7294_R_Tsense_Int);
	if (status)
		{
		return status;
		}
	i16Channel_Number = (i16TsenseInt_Reg&AD7294_Channel_Mask)>>12;
	fADC_Val = ((i16TsenseInt_Reg&AD7294_D9_to_D0_Mask)-(i16TsenseInt_Reg&AD7294_D10_Bit))/4.;
	fBTempInt[0] = fBTempInt[1];		// Load New Value into buffer
	fBTempInt[1] = fBTempInt[2];
	fBTempInt[2] = fADC_Val;
	fTempInt = (fTempInt*(i16average-1)+fBTempInt[0])/i16average;
	Median_Filter_for_3(&fBTempInt[0]);

	return status;

}
//*****************************************************************************
//
// AD7294 Sensor callback function.  Called at the end of AD7294 sensor driver
// transactions. This is called from I2C interrupt context. Therefore, we just
// set a flag and let main do the bulk of the computations and display.
//
//*****************************************************************************

/*
void AD7294AppCallback(void* pvCallbackData, uint_fast8_t ui8Status)
{
    if(ui8Status == I2CM_STATUS_SUCCESS)
    {
        g_vui8DataFlag = 1;
    }
}
*/
//*****************************************************************************
//
// Called by the NVIC as a result of I2C3 Interrupt. I2C3 is the I2C connection
// to the AD7294.
//
//*****************************************************************************
/*
void
AD7294I2CIntHandler(void)
{
    //
    // Pass through to the I2CM interrupt handler provided by sensor library.
    // This is required to be at application level so that I2CMIntHandler can
    // receive the instance structure pointer as an argument.
    //
    I2CMIntHandler(&g_sI2CInst);
}
*/


//*****************************************************************************
//
// Configure the UART and its pins.  This must be called before UARTprintf().
//
//*****************************************************************************
void
ConfigureUART(void)
{
    //
    // Enable the GPIO Peripheral used by the UART.
    //
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

    //
    // Enable UART0
    //
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);

    //
    // Configure GPIO Pins for UART mode.
    //
    ROM_GPIOPinConfigure(GPIO_PA0_U0RX);
    ROM_GPIOPinConfigure(GPIO_PA1_U0TX);
    ROM_GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    //
    // Use the internal 16MHz oscillator as the UART clock source.
    //
    UARTClockSourceSet(UART0_BASE, UART_CLOCK_PIOSC);

    //
    // Initialize the UART for console I/O.
    //
    UARTStdioConfig(0, 115200, 16000000);
}



//************************************************************************************
//initialize I2C module 3
//Slightly modified version of TI's example code
uint8_t InitI2C3(void)
{


    //enable I2C module 3
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C3);

    //reset module
    ROM_SysCtlPeripheralReset(SYSCTL_PERIPH_I2C3);

    //enable GPIO peripheral that contains I2C 3
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);

    // Configure the pin muxing for I2C3 functions on port B2 and B3.
    ROM_GPIOPinConfigure(GPIO_PD0_I2C3SCL);
    ROM_GPIOPinConfigure(GPIO_PD1_I2C3SDA);

    // Select the I2C function for these pins.
    ROM_GPIOPinTypeI2CSCL(GPIO_PORTD_BASE, GPIO_PIN_0);
    ROM_GPIOPinTypeI2C(GPIO_PORTD_BASE, GPIO_PIN_1);

    // Enable and initialize the I2C3 master module.  Use the system clock for
    // the I2C3 module.  The last parameter sets the I2C data transfer rate.
    // If false the data rate is set to 100kbps and if true the data rate will
    // be set to 400kbps.
    I2CMasterInitExpClk(I2C3_BASE, ROM_SysCtlClockGet(), false);

    //clear I2C FIFOs
    HWREG(I2C3_BASE + I2C_O_FIFOCTL) = 80008000;
    return 0;
}
//*****************************************************************************
uint8_t Init_AD7294()
{
    uint8_t i8Config_Reg_MSB=0;
    uint8_t i8Config_Reg_LSB=0;
    uint8_t i8Power_Down_Reg=0;
    uint16_t i16Config_Reg = 0;
    uint8_t	i8Sequence_Reg = 0;
    uint8_t	status = 0;

	status = I2CSend(AD7294_I2C_ADDRESS , 2, AD7294_RW_Power_Down,0xff);				// All channels powered off
    if (status)
    	{
    	return status;
    	}
    //
    // Delay about 10 milliseconds.
    //
    ROM_SysCtlDelay(ROM_SysCtlClockGet() / (10 * 30));
	status = I2CSend(AD7294_I2C_ADDRESS&0x0f , 2, AD7294_RW_Power_Down,0x00);		// All channels powered on (modified I2C Address)
    if (status)
    	{
    	return status;
    	}
	//
    // Delay about 10 milliseconds.
    //
    ROM_SysCtlDelay(ROM_SysCtlClockGet() / (10 * 30));
	status = I2CSend(AD7294_I2C_ADDRESS , 2, AD7294_RW_Power_Down,0x00);				// All channels powered on (regular I2C Address)
    if (status)
    	{
    	return status;
    	}
    //
    // Delay about 10 milliseconds.
    //
    ROM_SysCtlDelay(ROM_SysCtlClockGet() / (10 * 30));
	// Turn on all Bits in Sequence Register:
	i8Sequence_Reg = AD7294_Isense2_Seq_Bit + AD7294_Isense1_Seq_Bit + AD7294_Vin3_Seq_Bit + AD7294_Vin2_Seq_Bit + AD7294_Vin1_Seq_Bit + AD7294_Vin0_Seq_Bit;
	status = I2CSend(AD7294_I2C_ADDRESS , 2, AD7294_RW_Chan_Seq,i8Sequence_Reg);		//Set Config Register
    if (status)
    	{
    	return status;
    	}
    //
    // Delay about 10 milliseconds.
    //
    ROM_SysCtlDelay(ROM_SysCtlClockGet() / (10 * 30));
	//Turn on Auto Cycle Mode, Set Pin as Alert Output, All Vref are 2.5V:
	i8Config_Reg_MSB=AD7294_Auto_Cycle_Mode;
	i8Config_Reg_LSB=AD7294_Pin_is_Alert_Output+AD7294_Alert_Active_High;		//Pin is Alert Output, Polarity is active high
	status = I2CSend(AD7294_I2C_ADDRESS , 3, AD7294_RW_Config,i8Config_Reg_MSB,i8Config_Reg_LSB);		//Set Config Register
    if (status)
    	{
    	return status;
    	}
    //
    // Delay about 10 milliseconds.
    //
    ROM_SysCtlDelay(ROM_SysCtlClockGet() / (10 * 30));
	//Reading Test
	i8Power_Down_Reg=I2CReceive(AD7294_I2C_ADDRESS, AD7294_RW_Power_Down);

    //
    // Delay about 10 milliseconds.
    //
    ROM_SysCtlDelay(ROM_SysCtlClockGet() / (10 * 30));
	status = I2CReceive2(&i16Config_Reg, AD7294_I2C_ADDRESS ,AD7294_RW_Config);
	if (status)
		{
		return status;
		}

    UARTprintf("Power Down Register: 0x%x\t\t Configuration Register: 0x%x \n", i8Power_Down_Reg, i16Config_Reg);
    return 0;
}
//*****************************************************************************
void float_to_int_and_fract(float float_value, int32_t *i32IntegerPart, int32_t *i32FractionPart)
//
// Convert the floats to an integer part and fraction part for easy
// print.
//
{
*i32IntegerPart = (int32_t) float_value;
*i32FractionPart =(int32_t) (float_value * 1000.0f);
*i32FractionPart = *i32FractionPart - (*i32IntegerPart * 1000);
if(*i32FractionPart < 0)
	{
    *i32FractionPart *= -1;
	}
}

void delay_cycles(long cycles)
{
	long k = 0;

	while(k<cycles)
		{
		k = k+1;
		}
	k = k+5;
}
//*****************************************************************************
//
// Called by the NVIC as a SysTick interrupt, which is used to generate the
// sample interval
//
//*****************************************************************************
uint8_t SysTickIntHandler()
{
	uint8_t status = 0;
	if (SysTick_Semafore == 1)
	{
		status = 1;
		return status;
	}
	ROM_IntMasterDisable();
    //ROM_GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, GPIO_PIN_1);
	status = AD7294_Read_Results();
	if (status)
	{
		//return status;
	}
    //ROM_GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, 0x00);
    ROM_IntMasterEnable();
    SysTick_Semafore = 1;		// Indicate to Main Routine that Sys Tick occured

    return status;
}
//*****************************************************************************
//
// Main 'C' Language entry point.
//
//*****************************************************************************
int
main(void)
  {

    int32_t i32IntegerPart;
    int32_t i32FractionPart;
    uint8_t i8Count=0;
    uint8_t status;
    uint8_t k=0;
    uint8_t i8DAC_A_LSB=0;
    uint8_t i8DAC_A_MSB=0;
    uint16_t i16DAC_A=0;
    int32_t i32x;
    int32_t i32y;
    int my_rand;
    float my_noise;

    static char vin0_string[10];
    static char vin1_string[10];
    static char vin2_string[10];
    static char vin3_string[10];
    static char isense1_string[10];
    static char isense2_string[10];
    static char temp1_string[10];
    static char temp2_string[10];
    static char tempint_string[10];


    tContext sContext;
    tRectangle sRect, sRect2;

    //
    // The FPU should be enabled because some compilers will use floating-
    // point registers, even for non-floating-point code.  If the FPU is not
    // enabled this will cause a fault.  This also ensures that floating-
    // point operations could be added to this application and would work
    // correctly and use the hardware floating-point unit.  Finally, lazy
    // stacking is enabled for interrupt handlers.  This allows floating-
    // point instructions to be used within interrupt handlers, but at the
    // expense of extra stack usage.
    //
    FPUEnable();
    FPULazyStackingEnable();
   //
    // Setup the system clock to run at 40 Mhz from PLL with crystal reference
    //
    ROM_SysCtlClockSet(SYSCTL_SYSDIV_5 | SYSCTL_USE_PLL | SYSCTL_XTAL_16MHZ |
                       SYSCTL_OSC_MAIN);

    //ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    //GPIOPadConfigSet(GPIO_PORTB_BASE, 0xff, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD);

    //
    // Initialize the UART.
    //
    ConfigureUART();
    //
    // Print the welcome message to the terminal.
    //
    UARTprintf("\033[2JAD7294 Example\n");


    status = InitI2C3();


    Init_AD7294();

        //
        // Initialize the display driver.
        //
       Kentec320x240x16_SSD2119Init();





        //
        // Initialize the graphics context.
        //
        GrContextInit(&sContext, &g_sKentec320x240x16_SSD2119);

        //
        // Fill the top 24 rows of the screen with blue to create the banner.
        //
        sRect.i16XMin = 0;
        sRect.i16YMin = 0;
        sRect.i16XMax = GrContextDpyWidthGet(&sContext) - 1;
        sRect.i16YMax = 23;
        GrContextForegroundSet(&sContext, ClrRoyalBlue);
        GrRectFill(&sContext, &sRect);

        //
        // Put a white box around the banner.
        //
        GrContextForegroundSet(&sContext, ClrWhite);
        GrRectDraw(&sContext, &sRect);

        //
        // Put the application name in the middle of the banner.
        //
        GrContextFontSet(&sContext, &g_sFontCm20);
        GrStringDrawCentered(&sContext, "AD7294 test", -1,
                             GrContextDpyWidthGet(&sContext) / 2, 10, 0);
        //
        // Fill the remaining rows of the screen with green to create the banner.
        //
        sRect2.i16XMin = 0;
        sRect2.i16YMin = 24;
        sRect2.i16XMax = GrContextDpyWidthGet(&sContext) - 1;
        sRect2.i16YMax = GrContextDpyHeightGet(&sContext) - 1;
        GrContextForegroundSet(&sContext, ClrLime);
        GrRectFill(&sContext, &sRect2);

        //
        // Put a black box around the banner.
        //
        GrContextForegroundSet(&sContext, ClrBlack);
        GrContextBackgroundSet(&sContext, ClrLime);
        GrRectDraw(&sContext, &sRect);

        //
        // Start placing some strings
        //
        //GrContextFontSet(&sContext, &g_sFontCm20b);

        GrStringDraw(&sContext, "Vin0: ", -1,
                             (GrContextDpyWidthGet(&sContext) / 10)-5, 50, 0);
        GrStringDraw(&sContext, "Vin1: ", -1,
                             (GrContextDpyWidthGet(&sContext) / 10)-5, 80, 0);
        GrStringDraw(&sContext, "Vin2: ", -1,
                             (GrContextDpyWidthGet(&sContext) / 10)-5, 110, 0);
        GrStringDraw(&sContext, "Vin3: ", -1,
                             (GrContextDpyWidthGet(&sContext) / 10)-5, 140, 0);
        GrStringDraw(&sContext, "Isense1: ", -1,
                             (GrContextDpyWidthGet(&sContext) / 10)-25, 170, 0);
        GrStringDraw(&sContext, "Isense2: ", -1,
                             (GrContextDpyWidthGet(&sContext) / 10)-25, 200, 0);
        GrStringDraw(&sContext, "Temp1: ", -1,
                            ( GrContextDpyWidthGet(&sContext) / 2)+10, 50, 0);
        GrStringDraw(&sContext, "Temp2: ", -1,
                            ( GrContextDpyWidthGet(&sContext) / 2)+10, 80, 0);
        GrStringDraw(&sContext, "TempInt: ", -1,
                            ( GrContextDpyWidthGet(&sContext) / 2)-6, 110, 0);



        //
        // Configure and enable uDMA
        //
        SysCtlPeripheralEnable(SYSCTL_PERIPH_UDMA);
        SysCtlDelay(10);
        uDMAControlBaseSet(&sDMAControlTable[0]);
        uDMAEnable();


    //
    // Set the color to a white approximation.
    //
    g_pui32Colors[RED] = 0x8000;
    g_pui32Colors[BLUE] = 0x8000;
    g_pui32Colors[GREEN] = 0x8000;

    //
    // Initialize RGB driver. Use a default intensity and blink rate.
    //
    RGBInit(0);
    RGBColorSet(g_pui32Colors);
    RGBIntensitySet(0.5f);
    RGBEnable();

    //
    //Initial the GPIO for LED
    //
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    ROM_GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_1);
    ROM_GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, 0x00);

    //
    // Enable interrupts to the processor.
    //
    ROM_IntMasterEnable();





    // Enable the system ticks at 100 hz. - 100 measurements per second
    //
    uint32_t a = ROM_SysCtlClockGet();		// Current Clock Rate (40MHz)
    ROM_SysTickPeriodSet(a/3);			//Argument has to be <= 16777216
    ROM_SysTickIntEnable();
    ROM_SysTickEnable();

    //
    // After all the init and config we start blink the LED
    //
    //RGBBlinkRateSet(0.5f);




    //
    // Begin the data collection and printing.  Loop Forever.
    //
    while(1)
    {
    	i8Count++;
        //
        // Read the data from the AD7294 over I2C.  This command starts a
        // temperature measurement.  Then polls until temperature is ready.
        // Then automatically starts a pressure measurement and polls for that
        // to complete. When both measurement are complete and in the local
        // buffer then the application callback is called from the I2C
        // interrupt context.  Polling is done on I2C interrupts allowing
        // processor to continue doing other tasks as needed.
        //

    	// Just mess with the DAC A output for test purposes (saw tooth pattern):
    	//i8DAC_A_LSB = (i16DAC_A&0xff);
    	//i8DAC_A_MSB = (i16DAC_A >> 8)&0x0f;
    	//status = I2CSend(AD7294_I2C_ADDRESS , 3, AD7294_W_DAC_A,i8DAC_A_MSB,i8DAC_A_LSB);
    	//i16DAC_A++;


    	if (SysTick_Semafore==1) 	 // Only print after Sys Tick occured
    		{


    		//
    		// Create some noise
    		//
    		my_rand = urand();
    		my_noise = my_rand;
    		my_noise = my_noise/65536;
    		my_noise = my_noise/65536;
    		my_noise = my_noise-0.5;
        	//
            // Convert the floats to an integer part and fraction part for easy
            // print.
            //
    		fVin0 = 1.2 + my_noise/10;
    		float_to_int_and_fract(fVin0, &i32IntegerPart, &i32FractionPart);
            usprintf(vin0_string, "%2d.%02dV", i32IntegerPart, i32FractionPart);

    		fVin1 = 2.4+my_noise/10;
    		float_to_int_and_fract(fVin1, &i32IntegerPart, &i32FractionPart);
            usprintf(vin1_string, "%2d.%02dV ", i32IntegerPart, i32FractionPart);

    		fVin2 = 4.8+my_noise/10;
    		float_to_int_and_fract(fVin2, &i32IntegerPart, &i32FractionPart);
            usprintf(vin2_string, "%2d.%02dV ", i32IntegerPart, i32FractionPart);

    		fVin3 = 9.6+my_noise/10;
    		float_to_int_and_fract(fVin3, &i32IntegerPart, &i32FractionPart);
            usprintf(vin3_string, "%2d.%02dV ", i32IntegerPart, i32FractionPart);

            fIsense1 = 0.45+my_noise/10;
    		float_to_int_and_fract(fIsense1, &i32IntegerPart, &i32FractionPart);
            usprintf(isense1_string, "%2d.%02dA ", i32IntegerPart, i32FractionPart);

            fIsense2 = 0.56+my_noise/10;
    		float_to_int_and_fract(fIsense2, &i32IntegerPart, &i32FractionPart);
            usprintf(isense2_string, "%2d.%02dA ", i32IntegerPart, i32FractionPart);

            fTemp1 = 54+my_noise;
    		float_to_int_and_fract(fTemp1, &i32IntegerPart, &i32FractionPart);
            usprintf(temp1_string, "%3d.%01dC ", i32IntegerPart, i32FractionPart);

            fTemp2 = 96+my_noise;
    		float_to_int_and_fract(fTemp2, &i32IntegerPart, &i32FractionPart);
            usprintf(temp2_string, "%3d.%01dC ", i32IntegerPart, i32FractionPart);

            fTempInt = 120+my_noise;
    		float_to_int_and_fract(fTempInt, &i32IntegerPart, &i32FractionPart);
            usprintf(tempint_string, "%3d.%01dC ", i32IntegerPart, i32FractionPart);

            GrStringDraw(&sContext, vin0_string, 5, P11_x, P11_y, 1);
            GrStringDraw(&sContext, vin1_string, 5, P21_x, P21_y, 1);
            GrStringDraw(&sContext, vin2_string, 5, P31_x, P31_y, 1);
            GrStringDraw(&sContext, vin3_string, 5, P41_x, P41_y, 1);
            GrStringDraw(&sContext, isense1_string, 5, P51_x, P51_y, 1);
            GrStringDraw(&sContext, isense2_string, 5, P61_x, P61_y, 1);
            GrStringDraw(&sContext, temp1_string, 5, P12_x, P12_y, 1);
            GrStringDraw(&sContext, temp2_string, 5, P22_x, P22_y, 1);
            GrStringDraw(&sContext, tempint_string, 5, P32_x, P32_y, 1);

            UARTprintf("Vin0: %10s   ", vin0_string);
            UARTprintf("Vin1: %10s   ", vin1_string);
            UARTprintf("Vin2: %10s   ", vin2_string);
            UARTprintf("Vin3: %10s   ",vin3_string);
            UARTprintf("Isense1: %10s   ", isense1_string );
            UARTprintf("Isense2: %10s   ", isense2_string );
            UARTprintf("Temp1: %10s   ", temp1_string );
            UARTprintf("Temp2: %10s   ", temp2_string );
            UARTprintf("TempInt: %10s   ", tempint_string );
            UARTprintf("\n");

    		//SysTick_Semafore =0;
    		i8Count=0;

    	}

        //
        // Delay to keep printing speed reasonable. About 250 milliseconds.
        //
        //ROM_SysCtlDelay(ROM_SysCtlClockGet() / (4 * 3));
        //delay_cycles(timeout/100);

    }//while end
}


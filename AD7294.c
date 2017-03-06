//*****************************************************************************
//
// AD7294.c - Driver for the AD7294 pressure sensor.
//
// Copyright (c) 2012-2013 Texas Instruments Incorporated.  All rights reserved.
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
// This is part of revision 1.0 of the Tiva Firmware Development Package.
//
//*****************************************************************************

#include <math.h>
#include <stdint.h>
#include "sensorlib/hw_AD7294.h"
//#include "sensorlib/i2cm_drv.h"
#include "sensorlib/AD7294.h"

//*****************************************************************************
//
// The states of the AD7294 state machine.
//
//*****************************************************************************
#define AD7294_STATE_IDLE      0           // State machine is idle
#define AD7294_STATE_INIT1     1           // Waiting for initialization 1
#define AD7294_STATE_INIT2     2           // Waiting for initialization 2
#define AD7294_STATE_READ      3           // Waiting for read
#define AD7294_STATE_WRITE     4           // Waiting for write
#define AD7294_STATE_RMW       5           // Waiting for read-modify-write
#define AD7294_STATE_REQ_TEMP  6           // Requested temperature
#define AD7294_STATE_WAIT_TEMP 7           // Waiting for temperature ready
#define AD7294_STATE_READ_TEMP 8           // Reading temperature value
#define AD7294_STATE_REQ_PRES  9           // Requested pressure
#define AD7294_STATE_WAIT_PRES 10          // Waiting for pressure ready
#define AD7294_STATE_READ_PRES 11          // Reading pressure value


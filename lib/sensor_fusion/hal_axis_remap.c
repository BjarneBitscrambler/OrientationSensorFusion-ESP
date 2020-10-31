/*
 * Copyright (c) 2015, Freescale Semiconductor, Inc.
 * Copyright 2016-2017 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/*! \file hal_axis_remap.c
    \brief Hardware Abstraction layer for the particular sensors used.
	Depending on the design of the sensor PCB, the axes of the sensor ICs
	are not necessarily oriented as assumed by the fusion algorithm. This
	file defines the axis remapping functions that are always applied to
	the raw data before processing.

	The present remapping is appropriate for the FXOS8700/FXAS21002C
	sensor combination as found on the Adafruit breakout board.
	***to be verified***  copied from 
	FRDM_FXS_MULT2_B utilizing the FX0S8700 and FXAS21002 on the shield.

*/

#include "sensor_fusion.h"  // top level magCal and sensor fusion interfaces

void ApplyAccelHAL(struct AccelSensor *Accel)
{
	int8 i;				// loop counter

	// apply HAL to all measurements read from FIFO buffer
	for (i = 0; i < Accel->iFIFOCount; i++)
	{
		// apply HAL mapping to coordinate system used
#if THISCOORDSYSTEM == NED
		int16 itmp16 = Accel->iGsFIFO[i][CHX];
		Accel->iGsFIFO[i][CHX] = Accel->iGsFIFO[i][CHY];
		Accel->iGsFIFO[i][CHY] = itmp16;
#endif // NED
#if THISCOORDSYSTEM == ANDROID
		Accel->iGsFIFO[i][CHX] = -Accel->iGsFIFO[i][CHX];
		Accel->iGsFIFO[i][CHY] = -Accel->iGsFIFO[i][CHY];
#endif // Android
#if (THISCOORDSYSTEM == WIN8)
		Accel->iGsFIFO[i][CHZ] = -Accel->iGsFIFO[i][CHZ];
#endif // Win8

	} // end of loop over FIFO count

	return;
}

// function applies the hardware abstraction layer to the magnetometer readings
void ApplyMagHAL(struct MagSensor *Mag)
{
	int8 i;				// loop counter

	// apply HAL to all measurements read from FIFO buffer
	for (i = 0; i < Mag->iFIFOCount; i++)
	{
		// apply HAL mapping to coordinate system used
#if THISCOORDSYSTEM == NED
		int16 itmp16 = Mag->iBsFIFO[i][CHX];
		Mag->iBsFIFO[i][CHX] = -Mag->iBsFIFO[i][CHY];
		Mag->iBsFIFO[i][CHY] = -itmp16;
		Mag->iBsFIFO[i][CHZ] = -Mag->iBsFIFO[i][CHZ];
#endif // NED
#if THISCOORDSYSTEM == ANDROID
		Mag->iBsFIFO[i][CHX] = -Mag->iBsFIFO[i][CHX];
		Mag->iBsFIFO[i][CHY] = -Mag->iBsFIFO[i][CHY];
#endif // Android
#if THISCOORDSYSTEM == WIN8
		Mag->iBsFIFO[i][CHX] = -Mag->iBsFIFO[i][CHX];
		Mag->iBsFIFO[i][CHY] = -Mag->iBsFIFO[i][CHY];
#endif
	} // end of loop over FIFO count

	return;
}

// function applies the hardware abstraction layer to the gyro readings
void ApplyGyroHAL(struct GyroSensor *Gyro)
{
	int8 i;				// loop counter

	// apply HAL to all measurements read from FIFO buffer
	for (i = 0; i < Gyro->iFIFOCount; i++)
	{
		// apply HAL mapping to coordinate system used
#if THISCOORDSYSTEM == NED
		int16 itmp16 = Gyro->iYsFIFO[i][CHX];
		Gyro->iYsFIFO[i][CHX] = -Gyro->iYsFIFO[i][CHY];
		Gyro->iYsFIFO[i][CHY] = -itmp16;
		Gyro->iYsFIFO[i][CHZ] = -Gyro->iYsFIFO[i][CHZ];
#endif // NED
#if THISCOORDSYSTEM == ANDROID
		Gyro->iYsFIFO[i][CHX] = -Gyro->iYsFIFO[i][CHX];
		Gyro->iYsFIFO[i][CHY] = -Gyro->iYsFIFO[i][CHY];
#endif // Android
#if THISCOORDSYSTEM == WIN8
		Gyro->iYsFIFO[i][CHX] = -Gyro->iYsFIFO[i][CHX];
		Gyro->iYsFIFO[i][CHY] = -Gyro->iYsFIFO[i][CHY];
#endif // Win8

	} // end of loop over FIFO count

	return;
}
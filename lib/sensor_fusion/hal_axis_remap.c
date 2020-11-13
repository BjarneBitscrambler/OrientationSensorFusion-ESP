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

  With an unknown sensor board, the most straightforward way to determine
  the correct axis transformations is to use the Sensor Toolbox. Set
  the below transformations to your best guess (or start with no
  transformations). Load the software, launch the Toolbox, and then
  observe the orientation readouts while manipulating the board.
  Using the Digital Compass algorithm (which doesn't rely on accel
  or gyro), remap the axes as needed to get the Heading displaying
  correctly (i.e. near 0 degrees when your board is flat and pointing
  north, and the heading increases with clockwise rotation). Next adjust the
  accelerometer axes, using the Pitch and Roll readouts together with
  the Tilt-Compensated Compass algorithm. Finally, adjust the gyro
  axes by ensuring that rotations in all three axes give the expected
  response (i.e. that the magnetometer and accel are not 'fighting'
  the gyro, which causes the display to jerk around).

  NXP Application Note AN5017 explains the coordinate systems used.
  NED (North-East-Down) also called Aerospace, is the most appropriate
  for marine use. It is oriented such that when X-axis points north,
  the Y-axis points east, and the Z-axis points down. The fusion
  algorithm uses a "gravity-positive" interpretation - i.e. it expects
  each accelerometer axis to read +1g when pointing down, and -1g
  when pointing up. This is reversed from most accelerometer ICs.
  Rotations are considered positive when they are Clockwise as
  viewed in the increasing positive direction along the axis.
  Magnetometer readings are considered positive when the earth's
  magnetic field (from S to N) points in the same direction as the
  axis (i.e. the increasing direction of the axis points north).

  The present remapping is appropriate for the FXOS8700/FXAS21002C
  sensors on the Adafruit breakout board, with the NED coord system.
  Ax swapped with Ay.  
  Mx swapped with My, then Mx,My,Mz inverted.  
  Gx swapped with Gy, then Gx,Gy,Gz inverted.
  Checked 2020-11-13.

	The present remapping is appropriate for the FXOS8700/FXAS21002C
	sensors on the Adafruit breakout board, with the NED coord system.
	Accel-X, Mag-Y, Mag-Z, Gyro-Z all inverted. Checked 2020-11-06.
*/

#include "sensor_fusion.h"  // top level magCal and sensor fusion interfaces

// remap the Accelerometer axes
void ApplyAccelHAL(struct AccelSensor *Accel) {
  int8_t i;  // loop counter
  int16_t itmp16;

  // remap all measurements in FIFO buffer
  for (i = 0; i < Accel->iFIFOCount; i++) {
    // apply mapping for coordinate system used
#if THISCOORDSYSTEM == NED
		itmp16 = Accel->iGsFIFO[i][CHX];
		Accel->iGsFIFO[i][CHX] = Accel->iGsFIFO[i][CHY];
		Accel->iGsFIFO[i][CHY] = itmp16;
//    Accel->iGsFIFO[i][CHX] = -Accel->iGsFIFO[i][CHX];
//    Accel->iGsFIFO[i][CHY] = -Accel->iGsFIFO[i][CHY];
//    Accel->iGsFIFO[i][CHZ] = -Accel->iGsFIFO[i][CHZ];
#endif  // NED
#if THISCOORDSYSTEM == ANDROID
    // the ANDROID transformation has not been confirmed
#endif  // Android
#if (THISCOORDSYSTEM == WIN8)
    // the Windows transformation has not been confirmed
#endif  // Win8
  }  // end of loop over FIFO count
  return;
} // end ApplyAccelHAL()

// remap the Magnetometer axes
void ApplyMagHAL(struct MagSensor *Mag) {
  int8_t i;  // loop counter
  int16_t itmp16;

  // remap all measurements in FIFO buffer
  for (i = 0; i < Mag->iFIFOCount; i++) {
    // apply mapping for coordinate system used
#if THISCOORDSYSTEM == NED
		itmp16 = Mag->iBsFIFO[i][CHX];
    Mag->iBsFIFO[i][CHX] = -Mag->iBsFIFO[i][CHY];
		Mag->iBsFIFO[i][CHY] = -itmp16;
		Mag->iBsFIFO[i][CHZ] = -Mag->iBsFIFO[i][CHZ];
//    Mag->iBsFIFO[i][CHX] = -Mag->iBsFIFO[i][CHX];
//    Mag->iBsFIFO[i][CHY] = -Mag->iBsFIFO[i][CHY];
//    Mag->iBsFIFO[i][CHZ] = -Mag->iBsFIFO[i][CHZ];
#endif  // NED
#if THISCOORDSYSTEM == ANDROID
#endif  // Android
#if THISCOORDSYSTEM == WIN8
		Mag->iBsFIFO[i][CHX] = -Mag->iBsFIFO[i][CHX];
		Mag->iBsFIFO[i][CHY] = -Mag->iBsFIFO[i][CHY];
#endif
	} // end of loop over FIFO count
	return;
}//end ApplyMagHAL()


// remap the Gyroscope axes
void ApplyGyroHAL(struct GyroSensor *Gyro) {
  int8_t i;  // loop counter
  int16_t itmp16;
  // remap all measurements in FIFO buffer
  for (i = 0; i < Gyro->iFIFOCount; i++) {
    // apply mapping for coordinate system used
#if THISCOORDSYSTEM == NED
		itmp16 = Gyro->iYsFIFO[i][CHX];
		Gyro->iYsFIFO[i][CHX] = -Gyro->iYsFIFO[i][CHY];
		Gyro->iYsFIFO[i][CHY] = -itmp16;
		Gyro->iYsFIFO[i][CHZ] = -Gyro->iYsFIFO[i][CHZ];
//    Gyro->iYsFIFO[i][CHX] = -Gyro->iYsFIFO[i][CHX];
//    Gyro->iYsFIFO[i][CHY] = -Gyro->iYsFIFO[i][CHY];
//    Gyro->iYsFIFO[i][CHZ] = -Gyro->iYsFIFO[i][CHZ];
#endif  // NED
#if THISCOORDSYSTEM == ANDROID
#endif  // Android
#if THISCOORDSYSTEM == WIN8
		Gyro->iYsFIFO[i][CHX] = -Gyro->iYsFIFO[i][CHX];
		Gyro->iYsFIFO[i][CHY] = -Gyro->iYsFIFO[i][CHY];
#endif // Win8

	} // end of loop over FIFO count

	return;
}//end ApplyGyroHAL()

/*
 * Copyright (c) 2015, Freescale Semiconductor, Inc.
 * Copyright 2016-2017 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/*! \file build.h
    \brief Build configuration file

    This file contains only those parameters that directly relate to fusion
    implementation choices.  Board dependencies are in board.h.  Consult the
    Sensor Fusion User Guide for guidance and details.
*/

#ifndef BUILD_H
#define BUILD_H

#ifdef __cplusplus
extern "C" {
#endif

#define THISBUILD 720 ///< define build number sent in debug packet for display purposes only

// print debug messages to serial output. Set to 1 to enable, 0 to disable
#define ENABLE_DEBUG_LOG 1

/// @name CoordinateSystemBitFields
/// These defines determine the frame of reference (x, y, z axes and Euler angles) standard
/// to be used for a particular build.  Change THISCOORDSYSTEM to whichever of NED, ANDROID
/// or WIN8 you prefer.
///@{
#define NED             0   ///< identifier for NED (Aerospace) axes and angles
#define ANDROID         1   ///< identifier for Android axes and angles
#define WIN8            2   ///< identifier for Windows 8 axes and angles
#define THISCOORDSYSTEM NED ///< the coordinate system to be used
///@}

///@{
/// @name SensorBitFields
/// These bit-field values are used to declare which sensor types are used in the application.
/// Change bit-field values to 0x0000 for any features NOT USED.
/// These bitmasks are also used to set the pSensor->isInitialized flag once a particular
/// sensor is communicating successfully. F_USING_NONE indicates a problem with that sensor.
//TODO - unhandled exception if trying to not use gyro (and possibly others)
#define F_USING_NONE        0x0000 ///< 0x0000 indicates a sensor is unavailable / unconfigured.
#define F_USING_ACCEL       0x0001 ///< nominally 0x0001 if an accelerometer is to be used, 0x0000 otherwise
#define F_USING_MAG         0x0002 ///< nominally 0x0002 if an magnetometer  is to be used, 0x0000 otherwise
#define F_USING_GYRO        0x0004 ///< nominally 0x0004 if a gyro           is to be used, 0x0000 otherwise
#define F_USING_PRESSURE    0x0000 ///< nominally 0x0008 if altimeter        is to be used, 0x0000 otherwise
#define F_USING_TEMPERATURE 0x0000 ///< nominally 0x0010 if temp sensor      is to be used, 0x0000 otherwise
#define F_ALL_SENSORS       0x001F ///< refers to all applicable sensor types for the given physical unit
///@}
/// @name FusionSelectionBitFields
/// These bit-field values are used to declare which sensor fusion algorithms are used
/// in the application.  You can use more than one, although they all run from the same data.
/// Change individual bit-field values to 0x0000 for any features NOT USED.
///@{
#define F_1DOF_P_BASIC \
    0x0000 ///< 1DOF pressure (altitude) and temperature algorithm selector  - 0x0100 to include, 0x0000 otherwise
#define F_3DOF_G_BASIC \
    0x0200 ///< 3DOF accel tilt (accel) algorithm selector                   - 0x0200 to include, 0x0000 otherwise
#define F_3DOF_B_BASIC \
    0x0400 ///< 3DOF mag eCompass (vehicle/mag) algorithm selector           - 0x0400 to include, 0x0000 otherwise
#define F_3DOF_Y_BASIC \
    0x0800 ///< 3DOF gyro integration algorithm selector                     - 0x0800 to include, 0x0000 otherwise
#define F_6DOF_GB_BASIC \
    0x1000 ///< 6DOF accel and mag eCompass algorithm selector               - 0x1000 to include, 0x0000 otherwise
#define F_6DOF_GY_KALMAN \
    0x2000 ///< 6DOF accel and gyro (Kalman) algorithm selector              - 0x2000 to include, 0x0000 otherwise
#define F_9DOF_GBY_KALMAN \
    0x4000 ///< 9DOF accel, mag and gyro algorithm selector                  - 0x4000 to include, 0x0000 otherwise
///@}

/// @name SensorParameters
// The Output Data Rates (ODR) are set by the calls to *_Init() for each physical sensor.
// If a sensor has a FIFO, then it can be read once/fusion cycle; if not, then read more often
#define GYRO_ODR_HZ     400 ///< (int) requested gyroscope ODR Hz
#define ACCEL_ODR_HZ    200 ///< (int) requested accelerometer ODR Hz (overrides MAG_ODR_HZ for FXOS8700)
#define MAG_ODR_HZ      200 ///< (int) requested magnetometer ODR Hz (overridden by ACCEL_ODR_HZ for FXOS8700)
#define LOOP_RATE_HZ     40 //adjust according to the size of the FIFOs on sensors. If no FIFO (e.g. 
//FXOS8700 magnetometer) and don't want to skip any readings then need to read at same rate as ODR. 
//If FIFO exists or willing to skip readings, then usually set same as FUSION_HZ. See also sensor_fusion_class.h
#define FUSION_HZ       40  ///< (int) rate of fusion algorithm execution

// Output data rate parameters
#define MAXPACKETRATEHZ 40  //max rate at which data packets can practically be sent (e.g. to Fusion Toolbox)
#define RATERESOLUTION 1000 //When throttling back on output rate, this is the resolution in ms

//Specify which output method(s) to use for sending serial data packets and receiving commands
#define F_USE_WIRELESS_UART     0x0001	///< 0x0001 to include, 0x0000 otherwise
#define F_USE_WIRED_UART        0x0002	///< 0x0002 to include, 0x0000 otherwise

//#define INCLUDE_DEBUG_FUNCTIONS // Comment this line to disable the ApplyPerturbation function


#ifdef __cplusplus
}
#endif

#endif // BUILD_H

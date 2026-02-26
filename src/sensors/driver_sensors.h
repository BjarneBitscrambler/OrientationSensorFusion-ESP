/*
 * Copyright (c) 2015, Freescale Semiconductor, Inc.
 * Copyright (c) 2016-2017 NXP
 * Copyright (c) 2020 Bjarne Hansen
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/*! \file driver_sensors.h
    \brief Provides function prototypes for driver level interfaces
    It does not have a corresponding .c or .cpp file. Rather, it declares
    functions contained in hardware-specific files.
   
    To use, for example, the NXP FXOS8700, include the driver_FXOS8700.c source
    file in the build and ensure its Init and Read methods for the sensor are 
    declared here.

*/

#ifndef DRIVER_SENSORS_H
#define DRIVER_SENSORS_H

#include "driver_sensors_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/// @name Sensor Interface Prototypes
/// Each physical sensor must be provided with one initialization function
/// and one "read" function.  These must be installed by the user using the
/// installSensor method defined in SensorFusionGlobals.  A "physical sensor",
/// can mean either individual sensor type (such as a 3-axis accelerometer) or
/// a combo-sensor such as the NXP FXOS8700 6-axis accel plus mag. In the 
/// latter case, both the accel and mag readings will be made in the same call.
/// On the other hand, if you need the sensor modes read at different rates, 
/// then define a separate *_Read() function for each mode. This might be
/// prefered if, for example, the magnetometer doesn't have a FIFO whereas the 
/// the accelerometer on the same IC does and so the accelerometer can be read
/// in bursts less often than the magnetometer. The init()
/// function for each sensor is responsible for initializing all sensors contained
/// in that package.  The read() function is responsible for reading those same
/// sensors and moving the results into the standard structures contained within
/// the SensorFusionGlobals object.

#if defined(SENSOR_FXAX2100x_AND_FXOS8700)

#include "driver_fxos8700_registers.h"
#include "driver_fxos8700.h"
#include "driver_fxas21002.h"

#define Accel_Init FXOS8700_Accel_Init
#define Mag_Init   FXOS8700_Mag_Init
#define Therm_Init FXOS8700_Therm_Init
#define Gyro_Init  FXAS21002_Init

#define Accel_Read FXOS8700_Accel_Read
#define Mag_Read   FXOS8700_Mag_Read
#define Therm_Read FXOS8700_Therm_Read
#define Gyro_Read  FXAS21002_Read

/// @name SensorParameters
// The Output Data Rates (ODR) are set by the calls to *_Init() for each physical sensor.
// If a sensor has a FIFO, then it can be read once/fusion cycle; if not, then read more often
#define GYRO_ODR_HZ     400 ///< (int) requested gyroscope ODR Hz
#define ACCEL_ODR_HZ    200 ///< (int) requested accelerometer ODR Hz (overrides MAG_ODR_HZ for FXOS8700)
#define MAG_ODR_HZ      200 ///< (int) requested magnetometer ODR Hz (overridden by ACCEL_ODR_HZ for FXOS8700)

// sensor hardware details. The software FIFO is set to match the FIFO size defined here. It is possible
// to set a smaller software FIFO size (minimum is ODR / FUSION_HZ), but there may be extra accumulated data
// in the IC's FIFO that is lost/overrun.
#define GYRO_FIFO_SIZE  32	///< FXAX21000, FXAS21002 have 32 element FIFO
#define ACCEL_FIFO_SIZE 32	///< FXOS8700 (accel), MMA8652, FXLS8952 all have 32 element FIFO
#define MAG_FIFO_SIZE 	1	///< FXOS8700 (mag) and MAG3110 have no FIFO so equivalent to 1 element FIFO. For 
//these ICs we save 6 bytes * 31 = 186 bytes of RAM by setting this FIFO size to 1

#elif defined(SENSOR_LSM6DSOX_LIS3MDL)

#include "driver_lis3mdl.h"
#include "driver_lsm6dsox.h"

#define Accel_Init LSM6DSOX_Accel_Init
#define Mag_Init   LIS3MDL_Mag_Init
#define Therm_Init LSM6DSOX_Therm_Init
#define Gyro_Init  LSM6DSOX_Gyro_Init

#define Accel_Read LSM6DSOX_Accel_Read
#define Mag_Read   LIS3MDL_Mag_Read
#define Therm_Read LSM6DSOX_Therm_Read
#define Gyro_Read  LSM6DSOX_Gyro_Read

/// @name SensorParameters
// The Output Data Rates (ODR) are set by the calls to *_Init() for each physical sensor.
// If a sensor generates data faster than the fusion rate, then
// multiple data samples can be stored in the software FIFO. If a sensor has a hardware FIFO,
// it can be read once/fusion cycle; otherwise it should be read >= fusion rate.
#define GYRO_ODR_HZ     104 ///< (int) requested gyroscope ODR Hz
#define ACCEL_ODR_HZ    104 ///< (int) requested accelerometer ODR Hz (overrides MAG_ODR_HZ for FXOS8700)
#define MAG_ODR_HZ      155 ///< (int) requested magnetometer ODR Hz (overridden by ACCEL_ODR_HZ for FXOS8700)

// sensor hardware details. The software FIFO is set to match the FIFO size defined here. It is possible
// to set a smaller software FIFO size (minimum is ODR / FUSION_HZ), but there may be extra accumulated data
// in the IC's FIFO that is lost/overrun.
#define GYRO_FIFO_SIZE  (GYRO_ODR_HZ / FUSION_HZ + 1)	///< not using hardware FIFO on LSM6DSOX
#define ACCEL_FIFO_SIZE (ACCEL_ODR_HZ / FUSION_HZ + 1)	///< not using hardware FIFO on LSM6DSOX
#define MAG_FIFO_SIZE 	(MAG_ODR_HZ / FUSION_HZ + 1) ///< LIS3MDL (mag) has no hardware FIFO so minimum is  MAG_ODR_HZ / FUSION_HZ

#endif //checking which sensor hardware is used

#ifdef __cplusplus
}
#endif //__cplusplus

#endif // DRIVER_SENSORS_H

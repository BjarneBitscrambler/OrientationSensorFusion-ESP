/*
 * Copyright (c) 2015, Freescale Semiconductor, Inc.
 * Copyright 2016-2017 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/*! \file drivers.h
    \brief Provides function prototypes for driver level interfaces
    It does not have a corresponding .c or .cpp file. Rather, it declares
    functions contained in hardware-specific files.
   
    To use, for example, the NXP FXOS8700, include the driver_FXOS8700.c source
    file in the build and ensure its Init and Read methods for the sensor are 
    declared here.

   Modified by Bjarne Hansen, 2020-10-24
*/

#ifndef DRIVERS_H
#define DRIVERS_H

#ifdef __cplusplus
extern "C" {
#endif

// must #include "sensor_fusion.h" before this file

/// @name SysTick timing methods
/// The ARM SysTick routines were used on ARM M0+, M3,
/// M4 or M4F platforms time various fusion operations.  Timings
/// are then conveyed to and displayed by the NXP Sensor Fusion Toolbox.
/// They are not essential to the sensor fusion algorithm, but have
/// been redefined in hal_issdk.c for use in the Arduino environment.
///@{
extern void ARM_systick_enable(void);
extern void ARM_systick_start_ticks(int32_t *pstart);
extern int32_t ARM_systick_elapsed_ticks(int32_t start_ticks);
extern void ARM_systick_delay_ms(uint32_t iSystemCoreClock, uint32_t delay_ms);
///@}

/// @name Sensor Drivers
/// Each physical sensor must be provided with one initialization function
/// and one "read" function.  These must be installed by the user using the
/// installSensor method defined in SensorFusionGlobals.  By "physical sensor",
/// we mean either individual sensor type (such as a 3-axis accelerometer) or
/// a combo-sensor such as the NXP FXOS8700 6-axis accel plus mag.  The init()
/// function for each sensor is responsible for initializing all sensors contained
/// in that package.  The read() function is responsible for reading those same
/// sensors and moving the results into the standard structures contained within
/// the SensorFusionGlobals object.
///@{
//int8_t MPL3115_Init(struct PhysicalSensor *sensor, SensorFusionGlobals *sfg);
int8_t FXOS8700_Init(struct PhysicalSensor *sensor, SensorFusionGlobals *sfg);
int8_t FXAS21002_Init(struct PhysicalSensor *sensor, SensorFusionGlobals *sfg);
/*int8_t MMA8652_Init(struct PhysicalSensor *sensor, SensorFusionGlobals *sfg);
int8_t FXLS8952_Init(struct PhysicalSensor *sensor, SensorFusionGlobals *sfg);
int8_t MAG3110_Init(struct PhysicalSensor *sensor, SensorFusionGlobals *sfg);
int8_t MMA8451_Init(struct PhysicalSensor *sensor, SensorFusionGlobals *sfg);
int8_t FXLS8471Q_Init(struct PhysicalSensor *sensor, SensorFusionGlobals *sfg);
int8_t FXLS8962_Init(struct PhysicalSensor *sensor, SensorFusionGlobals *sfg);
int8_t FXLS8972_Init(struct PhysicalSensor *sensor, SensorFusionGlobals *sfg);
*/
//int8_t MPL3115_Read(struct PhysicalSensor *sensor, SensorFusionGlobals *sfg);
int8_t FXOS8700_Read(struct PhysicalSensor *sensor, SensorFusionGlobals *sfg);
int8_t FXAS21002_Read(struct PhysicalSensor *sensor, SensorFusionGlobals *sfg);
/*int8_t MMA8652_Read(struct PhysicalSensor *sensor, SensorFusionGlobals *sfg);
int8_t FXLS8952_Read(struct PhysicalSensor *sensor, SensorFusionGlobals *sfg);
int8_t MAG3110_Read(struct PhysicalSensor *sensor, SensorFusionGlobals *sfg);
int8_t MMA8451_Read(struct PhysicalSensor *sensor, SensorFusionGlobals *sfg);
int8_t FXLS8471Q_Read(struct PhysicalSensor *sensor, SensorFusionGlobals *sfg);
int8_t FXLS8962_Read(struct PhysicalSensor *sensor, SensorFusionGlobals *sfg);
int8_t FXLS8972_Read(struct PhysicalSensor *sensor, SensorFusionGlobals *sfg);
*/
//int8_t MPL3115_Idle(struct PhysicalSensor *sensor, SensorFusionGlobals *sfg);
int8_t FXOS8700_Idle(struct PhysicalSensor *sensor, SensorFusionGlobals *sfg);
int8_t FXAS21002_Idle(struct PhysicalSensor *sensor, SensorFusionGlobals *sfg);
/*int8_t MMA8652_Idle(struct PhysicalSensor *sensor, SensorFusionGlobals *sfg);
int8_t FXLS8952_Idle(struct PhysicalSensor *sensor, SensorFusionGlobals *sfg);
int8_t MAG3110_Idle(struct PhysicalSensor *sensor, SensorFusionGlobals *sfg);
int8_t MMA8451_Idle(struct PhysicalSensor *sensor, SensorFusionGlobals *sfg);
int8_t FXLS8471Q_Idle(struct PhysicalSensor *sensor, SensorFusionGlobals *sfg);
int8_t FXLS8962_Idle(struct PhysicalSensor *sensor, SensorFusionGlobals *sfg);
int8_t FXLS8972_Idle(struct PhysicalSensor *sensor, SensorFusionGlobals *sfg);
*/
///@}


#ifdef __cplusplus
}
#endif

#endif // DRIVERS_H

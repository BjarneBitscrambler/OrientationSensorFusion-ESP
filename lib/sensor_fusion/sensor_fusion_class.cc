/*
 * Copyright (c) 2020, Bjarne Hansen
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/*! \file sensor_fusion_class.cc
    \brief Class providing an easy-to-use interface to the
    NXP Sensor Fusion version 7 library algortihms.

    It is possible to directly interface with the library
    methods contained in the C files, most of which are
    based on those provided by NXP in their version 7.20 release. 
*/

#include "sensor_fusion_class.h"

#include <stdint.h>

#include "sensor_fusion.h"
#include "control.h"
#include "status.h"

SensorFusion::SensorFusion(int8_t pin_i2c_sda, int8_t pin_i2c_scl) {
  sfg_ = new SensorFusionGlobals();
  control_subsystem_ = new ControlSubsystem;
  status_subsystem_ = new StatusSubsystem;
  sensors_ = new PhysicalSensor
      [3];  // this implementation uses up to 3 sensors (accel/mag; gyro; baro/thermo)
            // TODO - can use std::vector as per
            // https://stackoverflow.com/questions/14114686/create-an-array-of-structure-using-new
            // which will allow adding arbitrary # of sensors.

  InitializeControlPort();
  InitializeStatusSubsystem();
  InitializeSensorFusionGlobals();

}  // end SensorFusion()

void SensorFusion::InitializeControlPort(void) {
  initializeControlPort(control_subsystem_);  // configure pins and ports for
                                              // the control sub-system
}  // end InitializeControlPort()

void SensorFusion::InitializeStatusSubsystem(void) {
  initializeStatusSubsystem(
      status_subsystem_);  // configure pins and ports for the status sub-system

}  // end InitializeStatusSubsystem()

void SensorFusion::InitializeSensorFusionGlobals(void) {
  initSensorFusionGlobals(sfg_, status_subsystem_, control_subsystem_);

}  // end InitializeSensorFusionGlobals()

void SensorFusion::InstallSensor( uint8_t sensor_i2c_addr, SensorType sensor_type ) {
// connect to the sensors we will be using.  Accelerometer and magnetometer are
// in same IC, and only requre one call.
// The sfg_ struct contains fields for  accel, magnetometer, gyro, baro, thermo.
// The *_Init() and *_Read() functions of each sensor are defined in driver_*.*
// files, and place data in the correct sfg_ fields

    switch (sensor_type) {
    case SensorType::kAccelerometer:
    case SensorType::kMagnetometer:
    case SensorType::kMagnetometerAccelerometer:
        sfg_->installSensor(sfg_, &sensors_[num_sensors_installed_], sensor_i2c_addr,
                            1, NULL, FXOS8700_Init, FXOS8700_Read);
        ++num_sensors_installed_;
        break;
    case SensorType::kGyroscope:
        sfg_->installSensor(sfg_, &sensors_[num_sensors_installed_], sensor_i2c_addr,
                            1, NULL, FXAS21002_Init, FXAS21002_Read);
        ++num_sensors_installed_;
        break;
    case SensorType::kBarometer:
    case SensorType::kThermometer:
        // TODO define some access functions for these
        // TODO add temperature/barometer sensor
        break;
    default:
        // unrecognized sensor type
        break;
    }

}  // end InstallSensor()


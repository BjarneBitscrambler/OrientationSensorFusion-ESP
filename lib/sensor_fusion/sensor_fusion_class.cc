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
#include "driver_sensors.h"
#include "status.h"

SensorFusion::SensorFusion(int8_t pin_i2c_sda, int8_t pin_i2c_scl) {
  sfg_ = new SensorFusionGlobals();
  control_subsystem_ = new ControlSubsystem;
  status_subsystem_ = new StatusSubsystem;
  sensors_ = new PhysicalSensor
      [MAX_NUM_SENSORS];  // this implementation uses up to 4 sensors (accel/mag; gyro; baro/thermo)
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

void SensorFusion::InitializeFusionEngine( ) {
     sfg_->initializeFusionEngine(sfg_);  // Initialize sensors and magnetic calibration
     sfg_->setStatus(sfg_, NORMAL);  // Set status state to NORMAL

} // end InitializeFusionEngine()

void SensorFusion::ReadSensors(void) {
    //Reads sensors.  Tracks how many invocations since last fusion of data, so
    //  sensors are read at various intervals (depending, e.g. on whether they have a fIFO)
    ++loops_per_fuse_counter_;

    sfg_->readSensors(sfg_, loops_per_fuse_counter_);  // Reads sensors, applies HAL, removes -32768 

}//end ReadSensors()

void SensorFusion::RunFusion(void) {
    //applies fusion algorithm to data accumulated in buffers
    
    sfg_->conditionSensorReadings(sfg_);  //Pre-processing; Magnetic calibration.
    sfg_->runFusion(sfg_);                // Run fusion algorithms
        
    // Serial.printf(" Algo took %ld us\n", sfg.SV_9DOF_GBY_KALMAN.systick);
    sfg_->loopcounter++;  // loop counter is used to "serialize" mag cal
                        // operations and blink LEDs to indicate status
    // This loop
    // should cycle at least four times for blink to operate
    // correctly. TODO - is this the best way?
    if (0 == sfg_->loopcounter % 4) {
      sfg_->updateStatus(sfg_);  // make pending status updates visible
    }

    sfg_->queueStatus(sfg_,NORMAL);  // assume NORMAL next pass through the loop
    //this resets temporary error conditions (SOFT_FAULT)
        
    loops_per_fuse_counter_ = 0;    //reset counter that determines when sensors are read

} // end RunFusion()

void SensorFusion::RunControlAndOutput(void) {
        //Make and send data to the Sensor Fusion Toolbox or whatever UART is connected to.
        //process any incoming commands
        sfg_->pControlSubsystem->stream(sfg_);  //create the output packet  
        sfg_->pControlSubsystem->write(sfg_->pControlSubsystem);  //send the output packet
        sfg_->pControlSubsystem->readCommands(sfg_);
}//end RunControlAndOutput()

float SensorFusion::GetHeadingDegrees(void) {
  return sfg_->SV_9DOF_GBY_KALMAN.fRhoPl;
}  // end GetHeadingDegrees()

bool SensorFusion::InstallSensor( uint8_t sensor_i2c_addr, SensorType sensor_type ) {
// connect to the sensors we will be using.  Accelerometer and magnetometer may
// be combined in the same IC, and only requre one call. If that is the case, then 
//  ensure the *_Read() function reads both the accel & magnetometer data.
// The sfg_ struct contains fields for  accel, magnetometer, gyro, baro, thermo.
// The *_Init() and *_Read() functions of each sensor are defined in driver_*.*
// files, and place data in the correct sfg_ fields

    if( num_sensors_installed_ >= MAX_NUM_SENSORS ) {
        //already have max number of sensors installed
      return false;
    }
    switch (sensor_type) {
    case SensorType::kAccelerometer:
        sfg_->installSensor(sfg_, &sensors_[num_sensors_installed_], sensor_i2c_addr,
                            kLoopsPerAccelRead, NULL, FXOS8700_Accel_Init, FXOS8700_Accel_Read);
        ++num_sensors_installed_;
        break;
    case SensorType::kMagnetometer:
      sfg_->installSensor(sfg_, &sensors_[num_sensors_installed_],
                          sensor_i2c_addr, kLoopsPerMagRead, NULL, FXOS8700_Mag_Init,
                          FXOS8700_Mag_Read);
      ++num_sensors_installed_;
      break;
    case SensorType::kMagnetometerAccelerometer:
      sfg_->installSensor(sfg_, &sensors_[num_sensors_installed_],
                          sensor_i2c_addr, kLoopsPerAccelRead, NULL, FXOS8700_Init,
                          FXOS8700_Read);
      ++num_sensors_installed_;
      break;
    case SensorType::kGyroscope:
        sfg_->installSensor(sfg_, &sensors_[num_sensors_installed_], sensor_i2c_addr,
                            kLoopsPerGyroRead, NULL, FXAS21002_Init, FXAS21002_Read);
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
    return true;
}  // end InstallSensor()

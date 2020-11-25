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
    methods contained in the C files, which are
    based on those provided by NXP in their version 7.20 release. 
*/

#include "sensor_fusion_class.h"

#include <stdint.h>
#include <Stream.h>

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

  InitializeControlSubsystem();
  InitializeStatusSubsystem();
  InitializeSensorFusionGlobals();

}  // end SensorFusion()

bool SensorFusion::InitializeControlSubsystem( const Stream *serial_port, const Stream *tcp_client) {

  return initializeControlPort(control_subsystem_, serial_port, tcp_client);  // configure pins and ports for
                                              // the control sub-system
}  // end InitializeControlSubsystem()

void SensorFusion::UpdateWiFiStream(void *tcp_client) {
  UpdateTCPClient(control_subsystem_, tcp_client);
}  // end UpdateTCPClient()

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
    // which is controlled by kLoopsPerMagRead, etc., in sensor_fusion_class.h
    
    sfg_->readSensors(sfg_, loops_per_fuse_counter_);  // Reads sensors, applies HAL, removes -32768 

}//end ReadSensors()

void SensorFusion::RunFusion(void) {
  // applies fusion algorithm to data accumulated in buffers

  // only run fusion every kLoopsPerFusionCalc'th time through loop
  if (loops_per_fuse_counter_ < kLoopsPerFusionCalc) {
    ++loops_per_fuse_counter_;
    return;
  }

  sfg_->conditionSensorReadings(sfg_);  // Pre-processing; Magnetic calibration.
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

  // assume NORMAL status next pass through the loop
  // this resets temporary error conditions (SOFT_FAULT)
  sfg_->queueStatus(sfg_,NORMAL);  

  loops_per_fuse_counter_ = 1;  // reset loop counter

}  // end RunFusion()

void SensorFusion::ProduceToolboxOutput(void) {
  // Make & send data to Sensor Fusion Toolbox or whatever UART is
  // connected to.
  if (loops_per_fuse_counter_ == 1) {       // only run if fusion has happened
    sfg_->pControlSubsystem->stream(sfg_);  // create output packet
    sfg_->pControlSubsystem->write(sfg_);   // send output packet
  }

}  // end ProduceToolboxOutput()

void SensorFusion::ProcessCommands(void) {
        //process any incoming commands
        sfg_->pControlSubsystem->readCommands(sfg_);
}//end ProcessCommands()

// The following Get____() methods return orientation values
// calculated by the 9DOF Kalman algorithm (the most advanced).
// They have been mapped to match the conventions used for
// vessels:
//  Compass Heading; 0 at magnetic north, and increasing CW.
//  Pitch; 0 with boat level, increasing with Bow Up.
//  Roll; 0 with boat level, increasing with Starboard roll.
//  Turn Rate; positive with turn to Starboard
//  Pitch Rate; positive with Bow moving Up.
//  Roll Rate; positive with increasing Starboard heel.
//  Acceleration; X to Bow, Y to Port, Z Up.
// This works with the Adafruit NXP FXOS8700/FXAS21002
//  breakout board, mounted with X toward the bow, Y to port,
//  and Z (component side of PCB) facing up.
// If the sensor orienatation is different than assumed, 
//  you may need to remap the axes below.  If a different
//  sensor board is used, you may need also to change the
//  axes mapping in the file hal_axis_remap.c  The latter
//  mapping is applied *before* the fusion algorithm, 
//  whereas the below mapping is applied *after*.

// fetch the Compass Heading in degrees
float SensorFusion::GetHeadingDegrees(void) {
//TODO - make generic so it's not dependent on algorithm used
  return sfg_->SV_9DOF_GBY_KALMAN.fRhoPl;
}  // end GetHeadingDegrees()

//fetch the Pitch in degrees
float SensorFusion::GetPitchDegrees(void) {
  return sfg_->SV_9DOF_GBY_KALMAN.fPhiPl;
}  // end GetPitchDegrees()

//fetch the Roll in degrees
float SensorFusion::GetRollDegrees(void) {
  return -(sfg_->SV_9DOF_GBY_KALMAN.fThePl);
}  // end GetRollDegrees()

float SensorFusion::GetTurnRateDegPerS(void) {
  return sfg_->SV_9DOF_GBY_KALMAN.fOmega[2];
}//end GetTurnRateDegPerS()

float SensorFusion::GetPitchRateDegPerS(void) {
  return sfg_->SV_9DOF_GBY_KALMAN.fOmega[0];
}//end GetTurnRateDegPerS()

float SensorFusion::GetRollRateDegPerS(void) {
  return -(sfg_->SV_9DOF_GBY_KALMAN.fOmega[1]);
}//end GetTurnRateDegPerS()

float SensorFusion::GetAccelXGees(void) {
  return sfg_->Accel.fGc[1];
}//end GetTurnRateDegPerS()

float SensorFusion::GetAccelYGees(void) {
  return sfg_->Accel.fGc[0];
}//end GetTurnRateDegPerS()

float SensorFusion::GetAccelZGees(void) {
  return sfg_->Accel.fGc[2];
}//end GetTurnRateDegPerS()

bool SensorFusion::InstallSensor(uint8_t sensor_i2c_addr,
                                   SensorType sensor_type) {
    // connect to the sensors we will be using.  Accelerometer and magnetometer
    // may be combined in the same IC, and only requre one call. If that is the
    // case, then
    //  ensure the *_Read() function reads both the accel & magnetometer data.
    // The sfg_ struct contains fields for  accel, magnetometer, gyro, baro,
    // thermo. The *_Init() and *_Read() functions of each sensor are defined in
    // driver_*.* files, and place data in the correct sfg_ fields

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
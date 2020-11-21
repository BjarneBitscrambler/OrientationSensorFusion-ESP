/*
 * Copyright (c) 2020 Bjarne Hansen
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause

 */

/*! \file sensor_fusion_class.h
    \brief Wrapper for the NXP Sensor Fusion v 7 functions.
*/

#ifndef SENSOR_FUSION_CLASS_H_
#define SENSOR_FUSION_CLASS_H_


#include "sensor_fusion.h"
#include "board.h"
#include "build.h"
#include "control.h"
#include "status.h"

enum class SensorType {
  kMagnetometer,
  kAccelerometer,
  kMagnetometerAccelerometer,
  kGyroscope,
  kBarometer,
  kThermometer
};

#define MAX_NUM_SENSORS  4    //TODO can replace with vector for arbitrary num sensors

class SensorFusion {
 public:
  SensorFusion(int8_t pin_i2c_sda = -1, int8_t pin_i2c_scl = -1);
  bool InstallSensor(uint8_t sensor_i2c_addr, SensorType sensor_type);
  void InitializeFusionEngine(void);
  void ReadSensors(void);
  void RunFusion(void);
  void RunControlAndOutput(void);
  float GetHeadingDegrees(void);

 private:
  void InitializeControlPort(void);
  void InitializeStatusSubsystem(void);
  void InitializeSensorFusionGlobals(void);

  SensorFusionGlobals *sfg_;             // Primary sensor fusion data structure
  ControlSubsystem *control_subsystem_;  // communications
  StatusSubsystem *status_subsystem_;    // visual status indicator
  PhysicalSensor *sensors_;              // up to 4 sensors
  uint8_t num_sensors_installed_ = 0;    //TODO calc this based on traversing sfg->pSensors->next till NULL
  
  uint8_t loops_per_fuse_counter_ = 0;   //counts how many sensor reads have been done between each fusion
  const uint8_t kLoopsPerMagRead = 1;     //how often a magnetometer read is performed
  const uint8_t kLoopsPerAccelRead = 5;   //how often an accelerometer read is performed
  const uint8_t kLoopsPerGyroRead = 5;    //how often a gyroscope read is performed
  const uint8_t kLoopsPerFusionCalc = 5;  //how often to fuse. Usually the max of previous 3 constants.

};  // end SensorFusion

#endif /* SENSOR_FUSION_CLASS_H_ */
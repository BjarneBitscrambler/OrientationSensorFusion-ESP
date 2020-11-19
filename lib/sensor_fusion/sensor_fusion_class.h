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

class SensorFusion {
 public:
  SensorFusion(int8_t pin_i2c_sda = -1, int8_t pin_i2c_scl = -1);
  void InstallSensor(uint8_t sensor_i2c_addr, SensorType sensor_type);

 private:
  void InitializeControlPort(void);
  void InitializeStatusSubsystem(void);
  void InitializeSensorFusionGlobals(void);

  SensorFusionGlobals *sfg_;             // Primary sensor fusion data structure
  ControlSubsystem *control_subsystem_;  // communications
  StatusSubsystem *status_subsystem_;    // visual status indicator
  PhysicalSensor *sensors_;              // up to 4 sensors
  uint8_t num_sensors_installed_ = 0;    //TODO calc this based on traversing sfg->pSensors->next till NULL

};  // end SensorFusion

#endif /* SENSOR_FUSION_CLASS_H_ */
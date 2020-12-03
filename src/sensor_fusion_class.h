/*
 * Copyright (c) 2020 Bjarne Hansen
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 */

/*! 
 *  \file sensor_fusion_class.h
 *
 *  Wrapper for the NXP Sensor Fusion v 7 functions.
 */

#ifndef SENSOR_FUSION_CLASS_H_
#define SENSOR_FUSION_CLASS_H_

#include <Stream.h>

#include "board.h"
#include "build.h"
#include "sensor_fusion/sensor_fusion.h"
#include "sensor_fusion/control.h"
#include "sensor_fusion/status.h"

/*!
 *  enum constants used to indicate what type of sensor is being installed
 *  when calling InstallSensor().
 */
enum class SensorType {
  kMagnetometer,
  kAccelerometer,
  kMagnetometerAccelerometer,
  kGyroscope,
  kBarometer,
  kThermometer
};

#define MAX_NUM_SENSORS  4    //TODO can replace with vector for arbitrary num sensors

/*!
 *  Class that wraps the various mostly-C-style functions of the
 *  sensor fusion code into easier to use methods. Not all the
 *  lower-level functions are exposed however; for more advanced
 *  use it may be necessary to call them directly.
 */
class SensorFusion {
  public:
    SensorFusion(int8_t pin_i2c_sda = -1, int8_t pin_i2c_scl = -1);
    bool InstallSensor(uint8_t sensor_i2c_addr, SensorType sensor_type);
    void InitializeFusionEngine(void);
    bool InitializeControlSubsystem( const Stream *serial_port = NULL, const Stream *tcp_client = NULL); 
    void UpdateWiFiStream(void *tcp_client); 
    void ReadSensors(void); 
    void RunFusion(void); 
    void ProduceToolboxOutput(void); 
    void ProcessCommands(void); 
    float GetHeadingDegrees(void); 
    float GetPitchDegrees(void); 
    float GetRollDegrees(void); 
    float GetAccelXGees(void);
    float GetAccelYGees(void);
    float GetAccelZGees(void);
    float GetTurnRateDegPerS(void);
    float GetPitchRateDegPerS(void);
    float GetRollRateDegPerS(void);

  private:
    void InitializeStatusSubsystem(void);
    void InitializeSensorFusionGlobals(void);

    SensorFusionGlobals *sfg_;             ///< Primary sensor fusion data structure
    ControlSubsystem *control_subsystem_;  ///< command and data streaming structure
    StatusSubsystem *status_subsystem_;    ///< visual status indicator structure
    PhysicalSensor *sensors_;              ///< linked list of up to 4 sensors
    uint8_t num_sensors_installed_ =
        0;  ///< tracks how many sensors have been added to list

  /*!
   * Constants of the form kLoopsPer_____ set the relationship between
   * number of sensor reads and each execution of the fusion algorithm.
   * Normally there is a 1:1 relationship (i.e. read, fuse, read, fuse,...)
   * but other arrangements are possible (e.g. read, read, fuse, read,
   * read,...) The rate at which main loop() executes is set by
   * LOOP_RATE_HZ in build.h
   */
    const uint8_t kLoopsPerMagRead =
        1;  ///< how often a magnetometer read is performed
    const uint8_t kLoopsPerAccelRead =
        1;  ///< how often an accelerometer read is performed
    const uint8_t kLoopsPerGyroRead =
        1;  ///< how often a gyroscope read is performed
    const uint8_t kLoopsPerFusionCalc =
        1;  ///< how often to fuse. Usually the max of previous 3 constants.
    uint8_t loops_per_fuse_counter_ =
        0;  ///< counts how many times through loop have been done

};  // end SensorFusion

#endif /* SENSOR_FUSION_CLASS_H_ */
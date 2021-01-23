/*
 * Copyright (c) 2020, Bjarne Hansen
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
/*!
 *  \file sensor_fusion_class.cc
 *
 *  \brief    An easy-to-use interface to the
 *  NXP Sensor Fusion version 7 library algorithms.
 *
 * It is configured to work with the Adafruit breakout board #3643
 * using the NXP FXOS8700 magnetometer/accelerometer and FXAS21002 gyroscope
 * sensor ICs, but can be modified to work with other sensors having an I2C
 * interface. With additional modification, it can also work with SPI interface
 * sensors.
 *
 *
 * A C++ class provides simple access to the most common sensor fusion
 * functions, but it is also possible to directly interface with the library
 * methods contained in the underlying C files, which are based on those
 * provided by NXP in their version 7.20 release. 
 */

#include "sensor_fusion_class.h"

#include <Stream.h>
#include <stdint.h>

#include "sensor_fusion/sensor_fusion.h"
#include "sensor_fusion/control.h"
#include "sensor_fusion/driver_sensors.h"
#include "sensor_fusion/status.h"

const float kDegToRads = PI / 180.0;  ///< To convert Degrees to Radians, multiply by this constant.
const float kCelsiusToKelvin = 273.15; ///< To convert degrees C to K, add this constant.
const float kGeesToMPerSS = 9.80665; ///< To convert acceleration in G to m/s^2, multiply by this constant.

/*!
 * Constructor creates and initializes a structure of variables used throughout
 * the functions. It initializes the control and status subsystems as well as
 * several subordinate data structures.
 */
SensorFusion::SensorFusion() {
  sfg_ = new SensorFusionGlobals();
  control_subsystem_ = new ControlSubsystem;
  status_subsystem_ = new StatusSubsystem;
  sensors_ = new PhysicalSensor
      [MAX_NUM_SENSORS];  // this implementation uses up to 4 sensors
                          // (accel/mag; gyro; baro/thermo)
                          // TODO - can use std::vector as per
                          // https://stackoverflow.com/questions/14114686/create-an-array-of-structure-using-new
                          // which will allow adding arbitrary # of sensors.

  InitializeInputOutputSubsystem();
  InitializeStatusSubsystem();
  InitializeSensorFusionGlobals();

}  // end SensorFusion()

/*!
 * Initialize the Control subsystem, which receives external commands and sends
 * data packets.
 * \param serial_port A Stream pointer to which to send output and receive commands. If
 * NULL, then output is not attempted.
 * \param tcp_client A WiFiClient pointer to which to send output and receive commands. If
 * NULL, then output is not attempted.
 * \return True if the Control subsystem is initialized, else False.
 */
bool SensorFusion::InitializeInputOutputSubsystem(const Stream *serial_port,
                                                  const void *tcp_client) {
  return initializeIOSubsystem(control_subsystem_, serial_port, tcp_client);
}  // end InitializeInputOutputSubsystem()

/*!
 * \brief Update the TCP client pointer.
 * Call when a new TCP connection is made, as reported by WiFiServer::available()
 * When tcp_client is NULL, output via WiFi is not attempted.
 * \param tcp_client A WiFiClient pointer used for input/output
 */
void SensorFusion::UpdateWiFiStream(void *tcp_client) {
  UpdateTCPClient(control_subsystem_, tcp_client);

}  // end UpdateTCPClient()

/*!
 * \brief Initialize the Status reporting system.
 * Status is indicated by LEDs, but other means can be added.
 */
void SensorFusion::InitializeStatusSubsystem(void) {
  initializeStatusSubsystem(
      status_subsystem_);  // configure the status sub-system

}  // end InitializeStatusSubsystem()

/*!
 * \brief Set the starting values of variables contained in sfg_
 */
void SensorFusion::InitializeSensorFusionGlobals(void) {
  initSensorFusionGlobals(sfg_, status_subsystem_, control_subsystem_);

}  // end InitializeSensorFusionGlobals()

/*!
 * Initialize the Sensors. Read calibrations. Set status to Normal.
 */
void SensorFusion::Begin(int pin_i2c_sda, int pin_i2c_scl) {
  sfg_->initializeFusionEngine(
      sfg_, pin_i2c_sda, pin_i2c_scl);                      // Initialize sensors and magnetic calibration
  sfg_->setStatus(sfg_, NORMAL);  // Set status state to NORMAL
//TODO - setStatus should check whether initialize worked.

}  // end InitializeFusionEngine()

/*!
 * \brief Reads all sensors.
 * Applies HAL remapping, removes invalid values, and stores data for 
 * later processing. Depending on whether a sensor has a built-in FIFO
 * buffer, that sensor may not be read each time this method is called.
 * See kLoopsPerMagRead, etc., in sensor_fusion_class.h
 */
void SensorFusion::ReadSensors(void) {
  sfg_->readSensors(
      sfg_,
      loops_per_fuse_counter_);  // Reads sensors, applies HAL, removes -32768

}  // end ReadSensors()

/*!
 * \brief Apply fusion algorithm to sensor raw data.
 * Sensor readings contained in global struct are calibrated and processed.
 * Status is updated and displayed.
 * Loop counter used for coordinating sensor reads is reset.
 * 
 */
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
  // LED Blinking is too fast unless status updates are slowed down.
  // Cycle at least four times per status update.
  // TODO - a better way might be to use a timer.
  if (0 == sfg_->loopcounter % 4) {
    sfg_->updateStatus(sfg_);  // make pending status updates visible
  }

  // assume NORMAL status next pass through the loop
  // this resets temporary error conditions (SOFT_FAULT)
  sfg_->queueStatus(sfg_, NORMAL);

  loops_per_fuse_counter_ = 1;  // reset loop counter

}  // end RunFusion()

/*!
 * \brief Generate and send out data, formatted for NXP Orientation Sensor Toolbox.
 * It is not mandatory to call this routine, if Toolbox output is not needed.
 */
void SensorFusion::ProduceToolboxOutput(void) {
  // Make & send data to Sensor Fusion Toolbox or whatever UART is
  // connected to.
  if (loops_per_fuse_counter_ == 1) {       // only run if fusion has happened
    sfg_->pControlSubsystem->stream(sfg_);  // create output packet
    sfg_->pControlSubsystem->write(sfg_);   // send output packet
  }

}  // end ProduceToolboxOutput()

/*!
 * places data from buffer into Control subsystem's output buffer, and sends
 * it out via serial and/or wifi.  Any existing data in the output buffer
 * that hasn't already been sent will be overwritten.
 * Returns true on success, false on problem such as data_length too long
 * for the transmit buffer.
 */
bool SensorFusion::SendArbitraryData(const char *buffer, uint16_t data_length) {
  if (data_length > MAX_LEN_SERIAL_OUTPUT_BUF) {
    return false;
  }
  char *out_buf = (char *)(sfg_->pControlSubsystem->serial_out_buf);
  for (uint16_t i = 0; i < data_length; i++) {
    out_buf[i] = buffer[i];
  }
  sfg_->pControlSubsystem->bytes_to_send = data_length;
  sfg_->pControlSubsystem->write(sfg_);  // send output packet
  return true;
}  // end SendArbitraryData()

/*!
 * \brief Process any incoming commands.
 * Commands may arrive by serial or WiFi connection, depending on which of
 * these is enabled (if any).
 * It is not mandatory to call this routine, if command responses are not needed.
 */
void SensorFusion::ProcessCommands(void) {
  // process any incoming commands
  sfg_->pControlSubsystem->readCommands(sfg_);
}  // end ProcessCommands()

/*!
 * \brief Inject and process a single command in the control subsystem.
 *
 * Command provided by this call bypasses the usual input route (serial
 * UART or WiFi) and is instead provided directly in this function call.
 * This provides an alterative way of manipulating the fusion algortithm
 * without needing to use the entire control or input subsystem, For a
 * list of valid commands, see control_input.c in the Orientation library.
 *
 * @param const char * command is a four-character sequence selected from
 * the list of valid commands. Illegal commands are ignored without
 * any action taken.
 */
void SensorFusion::InjectCommand(const char *command) {
  // process any incoming commands
  sfg_->pControlSubsystem->injectCommand(sfg_, (uint8_t *)command, 4);
}  // end InjectCommand()

/*!
 * \brief \return Boolean indicating whether orientation data are valid
 */
bool SensorFusion::IsDataValid(void) {
  if( NORMAL == sfg_->pStatusSubsystem->status ) {
    return true;
  } else {
    return false;
  }
}  // end IsDataValid()

/*!
 * \brief \return Fusion System status
 */
int SensorFusion::SystemStatus(void) {
  return (int)sfg_->pStatusSubsystem->status;
}  // end SystemStatus()

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

/*!
 * \brief \return Return the Compass Heading in degrees
 */
float SensorFusion::GetHeadingDegrees(void) {
  // TODO - make generic so it's not dependent on algorithm used
  return (sfg_->SV_9DOF_GBY_KALMAN.fRhoPl <= 90)
             ? (sfg_->SV_9DOF_GBY_KALMAN.fRhoPl + 270.0)
             : (sfg_->SV_9DOF_GBY_KALMAN.fRhoPl - 90.0);
}  // end GetHeadingDegrees()

/*!
 * \brief \return Return the Compass Heading in radians
 */
float SensorFusion::GetHeadingRadians(void) {
  return GetHeadingDegrees() * kDegToRads;
}  // end GetHeadingRadians()

/*!
 * \brief \return Return the Pitch in degrees
 */
float SensorFusion::GetPitchDegrees(void) {
  return sfg_->SV_9DOF_GBY_KALMAN.fPhiPl;
}  // end GetPitchDegrees()

/*!
 * \brief \return Return the Pitch in radians
 */
float SensorFusion::GetPitchRadians(void) {
  return GetPitchDegrees() * kDegToRads;
}  // end GetPitchRadians()

/*!
 * \brief \return Return the Roll in degrees
 */
float SensorFusion::GetRollDegrees(void) {
  return -(sfg_->SV_9DOF_GBY_KALMAN.fThePl);
}  // end GetRollDegrees()

/*!
 * \brief \return Return the Roll in radians
 */
float SensorFusion::GetRollRadians(void) {
  return GetRollDegrees() * kDegToRads;
}  // end GetRollRadians()

/*!
 * \brief \return Return the Temperature in degrees C
 */
float SensorFusion::GetTemperatureC(void) {
  return sfg_->Temp.temperatureC;
}  // end GetTemperatureC()

/*!
 * \brief \return Return the Temperature in degrees K
 */
float SensorFusion::GetTemperatureK(void) {
  return GetTemperatureC() + kCelsiusToKelvin;
}  // end GetTemperatureK()

/*!
 * \brief \return Return the Turn Rate in degrees
 */
float SensorFusion::GetTurnRateDegPerS(void) {
  return sfg_->SV_9DOF_GBY_KALMAN.fOmega[2];
}  // end GetTurnRateDegPerS()

/*!
 * \brief \return Return the Turn Rate in rad/s
 */
float SensorFusion::GetTurnRateRadPerS(void) {
  return GetTurnRateDegPerS() * kDegToRads;
}  // end GetTurnRateRadPerS()

/*!
 * \brief \return Return the Pitch Rate in degrees/s
 */
float SensorFusion::GetPitchRateDegPerS(void) {
  return sfg_->SV_9DOF_GBY_KALMAN.fOmega[0];
}  // end GetPitchRateDegPerS()

/*!
 * \brief \return Return the Pitch Rate in rad/s
 */
float SensorFusion::GetPitchRateRadPerS(void) {
  return GetPitchRateDegPerS() * kDegToRads;
}  // end GetPitchRateRadPerS()

/*!
 * \brief \return Return the Roll Rate in degrees/s
 */
float SensorFusion::GetRollRateDegPerS(void) {
  return -(sfg_->SV_9DOF_GBY_KALMAN.fOmega[1]);
}  // end GetRollRateDegPerS()

/*!
 * \brief \return Return the Roll Rate in rad/s
 */
float SensorFusion::GetRollRateRadPerS(void) {
  return GetRollRateDegPerS() * kDegToRads;
}  // end GetRollRateRadPerS()

/*!
 * \brief \return Return the X-axis Acceleration in gees
 */
float SensorFusion::GetAccelXGees(void) {
  return sfg_->Accel.fGc[1];
}  // end GetAccelXGees()

/*!
 * \brief \return Return the X-axis Acceleration in m/s^2
 */
float SensorFusion::GetAccelXMPerSS(void) {
  return GetAccelXGees() * kGeesToMPerSS;
}  // end GetAccelXMPerSS()

/*!
 * \brief \return Return the Y-axis Acceleration in gees
 */
float SensorFusion::GetAccelYGees(void) {
  return sfg_->Accel.fGc[0];
}  // end GetAccelYGees()

/*!
 * \brief \return Return the Y-axis Acceleration in m/s^2
 */
float SensorFusion::GetAccelYMPerSS(void) {
  return GetAccelYGees() * kGeesToMPerSS;
}  // end GetAccelYMPerSS()

/*!
 * \brief \return Return the Z-axis Acceleration in gees
 */
float SensorFusion::GetAccelZGees(void) {
  return sfg_->Accel.fGc[2];
}  // end GetAccelZGees()

/*!
 * \brief \return Return the Z-axis Acceleration in m/s^2
 */
float SensorFusion::GetAccelZMPerSS(void) {
  return GetAccelZGees() * kGeesToMPerSS;
}  // end GetAccelZMPerSS()

/*!
 * \brief Return the orientation as a quaternion
 * \param pointer to quaternion structure, to be filled by this method
 */
void  SensorFusion::GetOrientationQuaternion(Quaternion *quat) {
  quat->q0 = sfg_->SV_9DOF_GBY_KALMAN.fqPl.q0;
  quat->q1 = sfg_->SV_9DOF_GBY_KALMAN.fqPl.q1;
  quat->q2 = sfg_->SV_9DOF_GBY_KALMAN.fqPl.q2;
  quat->q3 = sfg_->SV_9DOF_GBY_KALMAN.fqPl.q3;
}  // end GetOrientationQuaternion()

/*!
 * \brief \return Return magnetic fit error of trial calibration
 */
float SensorFusion::GetMagneticFitErrorTrial(void) {
  return sfg_->MagCal.ftrFitErrorpc;
}  // end GetMagneticFitErrorTrial()

/*!
 * \brief \return Return magnetic fit error of current calibration
 */
float SensorFusion::GetMagneticFitError(void) {
  return sfg_->MagCal.fFitErrorpc;
}  // end GetMagneticFitError()


/*!
 * \brief \return Return geomagnetic field magnitude in uT of current calibration
 */
float SensorFusion::GetMagneticBMag(void) {
  return sfg_->MagCal.fB;
}  // end GetMagneticBMag()


/*!
 * \brief \return Return geomagnetic field magnitude in uT of trial calibration
 */
float SensorFusion::GetMagneticBMagTrial(void) {
  return sfg_->MagCal.ftrB;
}  // end GetMagneticBMagTrial()

/*!
 * \brief \return Return order [0,4,7,10] of trial calibration
 */
int32_t SensorFusion::GetMagneticCalOrderTrial(void) {
  return sfg_->MagCal.iValidMagCal;
}  // end GetMagneticCalOrderTrial()

/*!
 * \brief \return Return magnetic vector tilt error - X
 */
float SensorFusion::GetMagneticVectorTiltErrQ0(void) {
  return sfg_->SV_9DOF_GBY_KALMAN.fqmErrPl[0];
}  // end GetMagneticVectorTiltErrQ0()
/*!
 * \brief \return Return magnetic vector tilt error - Y
 */
float SensorFusion::GetMagneticVectorTiltErrQ1(void) {
  return sfg_->SV_9DOF_GBY_KALMAN.fqmErrPl[1];
}  // end GetMagneticVectorTiltErrQ1()
/*!
 * \brief \return Return magnetic fvector tilt error - Z
 */
float SensorFusion::GetMagneticVectorTiltErrQ2(void) {
  return sfg_->SV_9DOF_GBY_KALMAN.fqmErrPl[2];
}  // end GetMagneticVectorTiltErrQ2()

/*!
 * \brief Install Sensor in linked list
 * The max length of the list is checked, and if there is room, the 
 * given sensor is inserted at the head of the list.
 * An accelerometer and magnetometer may be combined in one IC - if
 * that is the case then only one call is required to install, 
 * provided the associated *_Init() and *_Read() function reads both
 * the accel & magnetometer data.  The Init() and Read() functions 
 * of each sensor are defined in driver_*.* files.
 * \param sensor_i2c_addr is the I2C bus address of the sensor IC
 * \param sensor_type indicates the type of sensor (e.g. magnetometer)
 * \return True if sensor installed successfully, else False
 */
bool SensorFusion::InstallSensor(uint8_t sensor_i2c_addr,
                                   SensorType sensor_type) {

    if( num_sensors_installed_ >= MAX_NUM_SENSORS ) {
        //already have max number of sensors installed
      return false;
    }
    switch (sensor_type) {
    case SensorType::kAccelerometer:
      sfg_->installSensor(sfg_, &sensors_[num_sensors_installed_],
                          sensor_i2c_addr, kLoopsPerAccelRead, NULL,
                          FXOS8700_Accel_Init, FXOS8700_Accel_Read);
      ++num_sensors_installed_;
      break;
    case SensorType::kMagnetometer:
      sfg_->installSensor(sfg_, &sensors_[num_sensors_installed_],
                          sensor_i2c_addr, kLoopsPerMagRead, NULL,
                          FXOS8700_Mag_Init, FXOS8700_Mag_Read);
      ++num_sensors_installed_;
      break;
    case SensorType::kMagnetometerAccelerometer:
      sfg_->installSensor(sfg_, &sensors_[num_sensors_installed_],
                          sensor_i2c_addr, kLoopsPerAccelRead, NULL,
                          FXOS8700_Init, FXOS8700_Read);
      ++num_sensors_installed_;
      break;
    case SensorType::kGyroscope:
      sfg_->installSensor(sfg_, &sensors_[num_sensors_installed_],
                          sensor_i2c_addr, kLoopsPerGyroRead, NULL,
                          FXAS21002_Init, FXAS21002_Read);
      ++num_sensors_installed_;
      break;
    case SensorType::kThermometer:
      // use the thermometer built into FXOS8700. Not precise nor calibrated,
      // but OK.
      sfg_->installSensor(sfg_, &sensors_[num_sensors_installed_],
                          sensor_i2c_addr, kLoopsPerThermRead, NULL,
                          FXOS8700_Therm_Init, FXOS8700_Therm_Read);
      ++num_sensors_installed_;
      break;
    case SensorType::kBarometer:
      // TODO define some access functions for this
      // TODO add barometer sensor
      break;
    default:
      // unrecognized sensor type
      break;
  }
  return true;
}  // end InstallSensor()

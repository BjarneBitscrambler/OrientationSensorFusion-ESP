/*
 * Copyright (c) 2015, Freescale Semiconductor, Inc.
 * Copyright 2016-2017 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/*! \file control.h
    \brief Defines control sub-system

   Each sensor fusion application will have its own set of functions
   to control the fusion process and report results.  This file defines the
   programming interface that should be followed in order for the fusion functions
   to operate correctly out of the box.  The actual command interpreter is
   defined separately in control_input.c.  The output streaming function
   is defined in control_output.c. Via these three files, the NXP Sensor Fusion
   Library provides a default set of functions which are compatible with the
   Sensor Fusion Toolbox.  Use of the toolbox is highly recommended at least
   during initial development, as it provides many useful debug features.
   The NXP development team will typically require use of the toolbox as a
   pre-requisite for providing software support.
   Data packets are sent via serial interface, which can be a wired UART, Bluetooth,
    socket via WiFi, etc.  Currently UART & WiFi are implemented.
*/

#ifndef _CONTROL_H_
#define _CONTROL_H_

#ifdef __cplusplus
extern "C" {
#endif

/// @name Control Port Function Type Definitions
/// "write" "stream" and "readCommands" provide three control functions visible at the main()
/// level.  These typedefs define the structure of those calls.
///@{
typedef int8_t (writePort_t) (SensorFusionGlobals *sfg);
typedef int8_t (readCommand_t) (SensorFusionGlobals *sfg);
typedef void (streamData_t)(SensorFusionGlobals *sfg);
///@}

/// \brief The ControlSubsystem encapsulates command and data streaming functions.
///
/// The ControlSubsystem encapsulates command and data streaming functions
/// for the library.  A C++-like typedef structure which includes executable methods
/// for the subsystem is defined here.
typedef struct ControlSubsystem {
	quaternion_type DefaultQuaternionPacketType;	///< default quaternion transmitted at power on
	volatile quaternion_type QuaternionPacketType;	///< quaternion type transmitted over UART
	volatile uint8_t AngularVelocityPacketOn;	///< flag to enable angular velocity packet
	volatile uint8_t DebugPacketOn;			///< flag to enable debug packet
	volatile uint8_t RPCPacketOn;			///< flag to enable roll, pitch, compass packet
	volatile uint8_t AltPacketOn;			///< flag to enable altitude packet
	volatile int8_t  AccelCalPacketOn;              ///< variable used to coordinate accelerometer calibration
    uint8_t         *serial_out_buf;        //buffer containing the output stream
    uint16_t        bytes_to_send;          //how many bytes waiting to go out
	writePort_t      *write;                        ///< function to write output buffer to the serial putput(s)
    readCommand_t    *readCommands;                 //< function to check for incoming commands and process them
    streamData_t     *stream;                       ///< function to create output data packets and place in buffer
} ControlSubsystem;

int8_t initializeControlPort(ControlSubsystem *pComm);  ///< Call this once to initialize structures, ports, etc.

// Located in output_stream.c:
/// Called once per fusion cycle to stream information required by the NXP Sensor Fusion Toolbox.
/// Packet protocols are defined in the NXP Sensor Fusion for Kinetis Product Development Kit User Guide.
void CreateOutgoingPackets(SensorFusionGlobals *sfg);

// Located in DecodeCommandBytes.c:
/// This function is responsible for decoding commands sent by the NXP Sensor Fusion Toolbox and setting
/// the appropriate flags in the ControlSubsystem data structure.
/// Packet protocols are defined in the NXP Sensor Fusion for Kinetis Product Development Kit User Guide.
void DecodeCommandBytes(SensorFusionGlobals *sfg, uint8_t input_buffer[], uint16_t nbytes);

/// Utility function used to place data in output buffer about to be transmitted via UART
void OutputBufAppendItem(uint8_t *pDest, uint16_t *pIndex, uint8_t *pSource, uint16_t iBytesToCopy);

#ifdef __cplusplus
}
#endif


#endif /* _CONTROL_H_ */

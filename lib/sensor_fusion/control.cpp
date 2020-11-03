/*
 * Copyright (c) 2015, Freescale Semiconductor, Inc.
 * Copyright 2016-2017 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/*! \file control.cpp
    \brief Defines control sub-system

    This file contains a UART implementation of the control subsystem.  The
    command interpreter and streaming functions are contained in two separate
    files.  So you can easily swap those out with only minor changes here.
*/
#include <Arduino.h>
#include <HardwareSerial.h>
#include <WiFi.h>
#include "sensor_fusion.h"
#include "control.h"
#include "build.h"

#define CONTROL_BAUDRATE        115200  ///< Baudrate to be used for serial communications

typedef int32_t status_t;   //from fsl_common.h

// global structures
uint8_t           sUARTOutputBuffer[256];             // larger than the nominal 124 byte size for outgoing packets

typedef enum {
    HardwareSerial,
    WiFiSerial
} UART_Type;

extern WiFiClient client; //defined in main.cpp

// Blocking function to write a single byte to a specified UART
void myUART_WriteByte(const UART_Type base, uint8_t data) {
  switch (base) {
    case HardwareSerial:
      Serial.write(data);
      break;
    case WiFiSerial:
      break;
    default:
      break;
  }
}  // end myUART_WriteByte()

// Blocking function to write multiple bytes to a specified output: a UART
//  or a TCP socket.
// TODO - we could interleave these for faster aggregate speed, rather than
//  waiting for each to finish.
// On ESP32, hardware UART has internal FIFO of length 0x7f, and once the
// bytes to be written are all in the FIFO, this routine returns. Actual
// sending of the data may take a while longer...
void myUART_WriteBytes(const UART_Type base, uint8_t *data, uint16_t byte_count) {
    uint16_t bytes_left = byte_count;
    int bytes_to_write;
  switch (base) {
    case HardwareSerial:
      while (bytes_left > 0) {
        bytes_to_write = Serial.availableForWrite();
        if( bytes_to_write > bytes_left ) {
          bytes_to_write = bytes_left;
        }
        Serial.write(&(data[byte_count - bytes_left]), bytes_to_write);
        bytes_left -= bytes_to_write;
      };
      break;
    case WiFiSerial:
      if (client.connected()) {
          // send data to wifi TCP socket
          uint16_t num_bytes_sent = 0;
          while (bytes_left > 0) {
            num_bytes_sent = client.write(&(data[byte_count - bytes_left]), bytes_left);
            bytes_left -= num_bytes_sent;
          }
        } else {
          client.stop();
        }
      break;
    default:
      break;
  }
}  // end myUART_WriteByte()

// Blocking function pipes specified buffer to both output UARTS
int8_t writeControlPort(ControlSubsystem *pComm, uint8_t buffer[], uint16_t nbytes)
{  
#if F_USE_WIRED_UART
        myUART_WriteBytes(HardwareSerial, buffer, nbytes);
#endif
#if F_USE_WIRELESS_UART
        myUART_WriteBytes(WiFiSerial, buffer, nbytes);
#endif
    return (0);
}

// Check for incoming commands, which are sequences of ASCII text,
// arriving on either hardware UART or TCP socket. Send them to
// function for decoding, as defined in DecodeCommandBytes.c
// Don't distinguish between which path the commands arrive by, 
// as it is unlikely one would have multiple sources at any one time.
int8_t ReceiveIncomingCommands(SensorFusionGlobals *sfg)
{
    uint8_t     data;

    //check for incoming bytes from serial UART
    while (0 < Serial.available() )
    {   data = Serial.read(); 
        DecodeCommandBytes(sfg, &data, 1);
    }
    // check for incoming bytes from TCP socket
    while (client.connected() && (0 < client.available())) {
        client.read(&data, 1);
        DecodeCommandBytes(sfg, &data, 1);
    }

    return 0;
}

/// Initialize the control subsystem and all related hardware
int8_t initializeControlPort(
    ControlSubsystem *pComm  ///< pointer to the control subystem structure
)
{
    if (pComm)
    {   //TODO - seems that many packet types are sent anyways. Check whether it's the Sensor Toolbox
        // that is overriding the defaults below.
        pComm->DefaultQuaternionPacketType = Q3;    // default to simplest algorithm
        pComm->QuaternionPacketType = Q3;           // default to simplest algorithm
        pComm->AngularVelocityPacketOn = false;      // transmit angular velocity packet
        pComm->DebugPacketOn = false;                // transmit debug packet
        pComm->RPCPacketOn = true;                  // transmit roll, pitch, compass packet
        pComm->AltPacketOn = false;                 // Altitude packet
        pComm->AccelCalPacketOn = false;
        pComm->write = writeControlPort;
        pComm->stream = CreateAndSendPackets;
        pComm->readCommands = ReceiveIncomingCommands;

        return (0);
    }
    else
    {
        return (1);
    }
}

/*
 * Copyright (c) 2015, Freescale Semiconductor, Inc.
 * Copyright 2016-2017 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/*! \file control.cpp
    \brief Defines control sub-system

    Contains methods to output data and receive commands. The physical transport
    is via either serial UART or WiFi, or both, depending on defines F_USE_WIRED_UART
    and F_USE_WIRELESS_UART. The command interpreter is located in DecodeCommandBytes.c
    The streaming functions that format the data into the output are in output_stream.c
*/
#include <Arduino.h>
#include <HardwareSerial.h>
#ifdef ESP8266
  #include <ESP8266WiFi.h>
#endif
#ifdef ESP32
  #include <WiFi.h>
#endif
#include "sensor_fusion.h"
#include "control.h"
#include "build.h"

// global structures
uint8_t           sUARTOutputBuffer[256];  // larger than the nominal 124 byte size for outgoing packets

typedef enum {
    HardwareSerial,
    WiFiSerial
} UART_Type;

//TODO - can start WiFi client in initializeControlPort() instead of global
extern WiFiClient client; //defined in main.cpp

// Blocking function to write multiple bytes to specified output(s): a UART
//  or a TCP socket, as specified in build.h
// On ESP32, hardware UART has internal FIFO of length 0x7f, and once the
// bytes to be written are all in the FIFO, this routine returns. Actual
// sending of the data may take a while longer...
int8_t SendSerialBytesOut(ControlSubsystem *pComm)
{  
    //track the number of bytes separately and run wired/wireless output in parallel
#if F_USE_WIRED_UART
    uint16_t bytes_left_wired = pComm->bytes_to_send;
#else
    uint16_t bytes_left_wired = 0;
#endif
#if F_USE_WIRELESS_UART
    uint16_t bytes_left_wireless = pComm->bytes_to_send;
#else
    uint16_t bytes_left_wireless = 0;
#endif
    int bytes_to_write_wired;

    while ((bytes_left_wired > 0) || (bytes_left_wireless > 0)) {
      if (bytes_left_wired > 0) {
        bytes_to_write_wired = Serial.availableForWrite();
        if( bytes_to_write_wired > bytes_left_wired ) {
          bytes_to_write_wired = bytes_left_wired;
        }
        //write() won't return until all requested are sent, so only ask for what there's room for
        Serial.write(&(pComm->serial_out_buf[pComm->bytes_to_send - bytes_left_wired]), bytes_to_write_wired);
        bytes_left_wired -= bytes_to_write_wired;
      };
      if (client.connected() && (bytes_left_wireless > 0)) {
        // send data to wifi TCP socket.  write() returns actual # queued, which may be less than requested.
          bytes_left_wireless -= client.write(&(pComm->serial_out_buf[pComm->bytes_to_send - bytes_left_wireless]), bytes_left_wireless);
      }else if( !client.connected()) {
        client.stop();
        bytes_left_wireless = 0;  //don't bother trying to send any remaining bytes
      }
    }//end while() there are unsent bytes
    pComm->bytes_to_send = 0;
    return (0);
}//end SendSerialBytesOut()

// Check for incoming commands, which are sequences of ASCII text,
// arriving on either hardware UART or TCP socket. Send them to
// function for decoding, as defined in DecodeCommandBytes.c
// Doesn't distinguish between which path the commands arrive by, 
// as it is unlikely one would have multiple simultaneous sources.
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
}//end ReceiveIncomingCommands()

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
        pComm->serial_out_buf = sUARTOutputBuffer;
        pComm->write = SendSerialBytesOut;
        pComm->stream = CreateOutgoingPackets;
        pComm->readCommands = ReceiveIncomingCommands;

        return (0);
    }
    else
    {
        return (1);
    }
}//end initializeControlPort()

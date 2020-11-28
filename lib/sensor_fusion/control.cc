/*
 * Copyright (c) 2015, Freescale Semiconductor, Inc.
 * Copyright 2016-2017 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/*! \file control.cc
    \brief Defines control sub-system

    Contains methods to output data and receive commands. The physical transport
    is via either serial UART or WiFi, or both, depending on defines F_USE_WIRED_UART
    and F_USE_WIRELESS_UART in build.h, and on arguments to initializeIOSubsystem(). 
    The command interpreter is located in control_input.c
    The streaming functions that format the data into the output are in control_output.c
*/
#include <Arduino.h>
#include <HardwareSerial.h>
#ifdef ESP8266
  #include <ESP8266WiFi.h>
#endif
#ifdef ESP32
  #include <WiFi.h>
#endif
#include "sensor_fusion.h" // Requires sensor_fusion.h to occur first in the #include stackup
#include "build.h"
#include "control.h"

// global structures
uint8_t sUARTOutputBuffer[MAX_LEN_SERIAL_OUTPUT_BUF];

// Blocking function to write multiple bytes to specified output(s): a UART
//  or a TCP socket
// On ESP32, hardware UART has internal FIFO of length 0x7f, and once the
// bytes to be written are all in the FIFO, this routine returns. Actual
// sending of the data may take a while longer...
int8_t SendSerialBytesOut(SensorFusionGlobals *sfg)
{
  ControlSubsystem *pComm = sfg->pControlSubsystem;
  // track number of bytes separately to run wired/wireless output in parallel
    uint16_t bytes_left_wired = 0;
    HardwareSerial *serial_port = (HardwareSerial *) (pComm->serial_port);
    if (serial_port) {
      bytes_left_wired = pComm->bytes_to_send;
    }

    WiFiClient *tcp_client = (WiFiClient *)(pComm->tcp_client);
    uint16_t bytes_left_wireless = 0;
    if (tcp_client) {
      bytes_left_wireless = pComm->bytes_to_send;
    }

    int bytes_to_write_wired;

    while ((bytes_left_wired > 0) || (bytes_left_wireless > 0)) {
      if (bytes_left_wired > 0) {
        bytes_to_write_wired = serial_port->availableForWrite();
        if( bytes_to_write_wired > bytes_left_wired ) {
          bytes_to_write_wired = bytes_left_wired;
        }
        //write() won't return until all requested are sent, so only ask for what there's room for
        serial_port->write(&(pComm->serial_out_buf[pComm->bytes_to_send - bytes_left_wired]), bytes_to_write_wired);
        bytes_left_wired -= bytes_to_write_wired;
      };
      if (tcp_client->connected() && (bytes_left_wireless > 0)) {
        // send data to wifi TCP socket.  write() returns actual # queued, which may be less than requested.
          bytes_left_wireless -= tcp_client->write(&(pComm->serial_out_buf[pComm->bytes_to_send - bytes_left_wireless]), bytes_left_wireless);
      }else if( !tcp_client->connected()) {
        tcp_client->stop();
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
    WiFiClient *tcp_client = (WiFiClient *) sfg->pControlSubsystem->tcp_client;
    HardwareSerial *serial_port = (HardwareSerial*) sfg->pControlSubsystem->serial_port;

    // check for incoming bytes from serial UART
    if( serial_port ) {
        while (0 < Serial.available() )
      {   data = serial_port->read(); 
          DecodeCommandBytes(sfg, &data, 1);
      }
    }
    // check for incoming bytes from TCP socket
    if (tcp_client) {
      while (tcp_client->connected() && (0 < tcp_client->available())) {
        tcp_client->read(&data, 1);
        DecodeCommandBytes(sfg, &data, 1);
      }
    }

    return 0;
}//end ReceiveIncomingCommands()

/// Initialize the control subsystem and all related hardware
bool initializeIOSubsystem(
    ControlSubsystem *pComm,  ///< pointer to the control subystem structure
    const void *serial_port, const void *tcp_client )
{
    if (pComm)
    { //commands (e.g. from Sensor Toolbox) can change some of these, such as 
      //which packets are enabled
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
        pComm->serial_port = serial_port;     
        pComm->tcp_client = tcp_client;

        return true;
    }
    else
    {
        return false;
    }
}//end initializeIOSubsystem()

void UpdateTCPClient(ControlSubsystem *pComm,void *tcp_client) {
    pComm->tcp_client = tcp_client;
}//end UpdateTCPClient()

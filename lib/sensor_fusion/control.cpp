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
#include "sensor_fusion.h"
#include "control.h"

#ifndef F_USE_WIRELESS_UART
#define F_USE_WIRELESS_UART     0x0000	///< 0x0001 to include, 0x0000 otherwise
#endif
#ifndef F_USE_WIRED_UART
#define F_USE_WIRED_UART        0x0002	///< 0x0002 to include, 0x0000 otherwise
#endif

#define CONTROL_BAUDRATE        115200  ///< Baudrate to be used for serial communications

/* Forward declaration of the handle typedef. */
typedef struct _uart_handle uart_handle_t;

//typedef uint16_t size_t;  //already defined in stddef.h
typedef int32_t status_t;   //from fsl_common.h

/*! @brief UART handle structure. */
struct _uart_handle
{
    uint8_t *volatile txData;   /*!< Address of remaining data to send. */
    volatile size_t txDataSize; /*!< Size of the remaining data to send. */
    size_t txDataSizeAll;       /*!< Size of the data to send out. */
    uint8_t *volatile rxData;   /*!< Address of remaining data to receive. */
    volatile size_t rxDataSize; /*!< Size of the remaining data to receive. */
    size_t rxDataSizeAll;       /*!< Size of the data to receive. */

    uint8_t *rxRingBuffer;              /*!< Start address of the receiver ring buffer. */
    size_t rxRingBufferSize;            /*!< Size of the ring buffer. */
    volatile uint16_t rxRingBufferHead; /*!< Index for the driver to store received data into ring buffer. */
    volatile uint16_t rxRingBufferTail; /*!< Index for the user to get data from the ring buffer. */

//    uart_transfer_callback_t callback; /*!< Callback function. */
    void *userData;                    /*!< UART callback function parameter.*/

    volatile uint8_t txState; /*!< TX transfer state. */
    volatile uint8_t rxState; /*!< RX transfer state */
};

uart_handle_t   wired_uartHandle;
uart_handle_t   wireless_uartHandle;

// global structures
uint8_t           sUARTOutputBuffer[256];             // larger than the nominal 124 byte size for outgoing packets

// direct access to sfg here is the only place in the entire library where we cannot simply
// pass a pointer.  This is because it is needed by the UART interrupt handlers.  Since this
// only occurs here, in a subsystem which is defined to be application dependent, that is
// considered acceptable.
extern SensorFusionGlobals sfg;

typedef enum {
    HardwareSerial,
    WiFiSerial
} UART_Type;

const UART_Type hardware_serial = HardwareSerial;
const UART_Type wifi_serial = WiFiSerial;
#define WIRED_UART hardware_serial
#define WIRELESS_UART wifi_serial

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

// Blocking function to write a multiple bytes to a specified UART
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
      break;
    default:
      break;
  }
}  // end myUART_WriteByte()

// Blocking function pipes specified buffer to both output UARTS
int8_t writeControlPort(ControlSubsystem *pComm, uint8_t buffer[], uint16_t nbytes)
{  
#if F_USE_WIRED_UART
        myUART_WriteBytes(WIRED_UART, buffer, nbytes);
#endif
#if F_USE_WIRELESS_UART
        myUART_WriteByte(WIRELESS_UART, buffer[i]);
#endif
    return (0);
}

#if F_USE_WIRELESS_UART
// writeWirelessPort() is called from BlueRadios_Init(), which is used to
// initialize the Bluetooth module on NXP sensor shields.  That function will
// obviously need to be replaced for alternate hardware.
int8_t writeWirelessPort(uint8_t buffer[], uint16_t nbytes)
{
    uint16_t    i;
    for (i = 0; i < nbytes; i++)
    {
        myUART_WriteByte(WIRELESS_UART, buffer[i]);
    }

    return (0);
}


// Wired and Wireless UART interrupt handlers are essentially identical.
void WIRELESS_UART_IRQHandler(void)
{
/*    uint8_t     data;
    status_t    sts;
    uint32_t    nbytes;				// number of bytes received
    uint32_t    flags;
    static char iCommandBuffer_B[5] = "~~~~";	// 5 bytes long to include the unused terminating \0
    sfg.setStatus(&sfg, RECEIVING_WIRELESS);
    flags = UART_GetStatusFlags(WIRELESS_UART);
    // If new data arrived. 
    if ((kUART_RxDataRegFullFlag | kUART_RxOverrunFlag) & flags)
    {
        sts = UART_TransferGetReceiveCount(WIRELESS_UART, &wireless_uartHandle, &nbytes);
        if (sts == kStatus_Success)
        {
            data = UART_ReadByte(WIRELESS_UART);
            DecodeCommandBytes(&sfg, iCommandBuffer_B, &data, 1);
        }
    }
    */
}
// initialize BlueRadios BR-LE4.0-D2A Bluetooth module
// This is required for NXP FRDM-FXS-MULT2-B boards.
void BlueRadios_Init(void)
{
}

#endif  //F_USE_WIRELESS_UART

#if F_USE_WIRED_UART
void echo(uint8_t data)  // only used for comms debug
{
    // Send data only when UART TX register is empty and ring buffer has data to send out.
    //bj changed so if there is room, rather than empty
    if ( 0 < Serial.availableForWrite())
    {   
        myUART_WriteByte(WIRED_UART, data);
    }
}
// Wired and Wireless UART interrupt handlers are essentially identical.
void WIRED_UART_IRQHandler(void)
{
    uint8_t     data;
//    status_t    sts;
//    uint32_t    nbytes;		   // number of bytes received
//    uint32_t    flags;
    static char iCommandBuffer_A[5] = "~~~~";	// 5 bytes long to include the unused terminating \0

    sfg.setStatus(&sfg, RECEIVING_WIRED);
    //flags = UART_GetStatusFlags(WIRED_UART);
    //If new data arrived.
    if (0 < Serial.available() )
    {   //data = UART_ReadByte(WIRED_UART);
      data = Serial.read(); //TODO - does this lose the ability to read from wireless?
      DecodeCommandBytes(&sfg, iCommandBuffer_A, &data, 1);
    }
}
#endif

/// Initialize the control subsystem and all related hardware
// bj removed UART (wired & wireless) initialization - perform this in main
int8_t initializeControlPort(
    ControlSubsystem *pComm  ///< pointer to the control subystem structure
)
{
    if (pComm)
    {
        pComm->DefaultQuaternionPacketType = Q3;    // default to simplest algorithm
        pComm->QuaternionPacketType = Q3;           // default to simplest algorithm
        pComm->AngularVelocityPacketOn = true;      // transmit angular velocity packet
        pComm->DebugPacketOn = true;                // transmit debug packet
        pComm->RPCPacketOn = true;                  // transmit roll, pitch, compass packet
        pComm->AltPacketOn = true;                 // Altitude packet
        pComm->AccelCalPacketOn = 0;
        pComm->write = writeControlPort;
        pComm->stream = CreateAndSendPackets;

        return (0);
    }
    else
    {
        return (1);
    }
}

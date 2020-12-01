/*
 * Copyright (c) 2015 - 2016, Freescale Semiconductor, Inc.
 * Copyright (c) 2016-2017 NXP
 * Copyright (c) 2020 Bjarne Hansen
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 *  Modified for Espressif ESP environment
 *  Fusion library requires methods to control and read from physical sensor ICs. These methods are
 *  found in files like FXAS21002.c and FXOS8700.c.  For example,
 *  driver_FXAS21002.c implements methods like FXAS21002_Init() using calls to Sensor_I2C_Write_List().
 *  This present file provides a Sensor_I2C_Write_List() that functions in the ESP environment.
 */

/**
 * @file hal_i2c.cc
 * @brief Contains definitions for low-level interface functions
 *  for reading and writing data from/to sensor using I2C.
 */

#include "Arduino.h"
#include <Wire.h>
#include "driver_sensors_types.h"
#include "hal_i2c.h"


/**************************************************************************/
/*!
    @brief  Initialize the I2C system at max clock rate supported by sensors.
    pin_sda and pin_scl indicate the pin numbers to which the I2C SDA and SCL
    lines of the sensors are connected. Pass -1 to use the default Arduino pins.
    
    Returns true if successful, false if problem initializing I2C.
*/
/**************************************************************************/
bool I2CInitialize( int pin_sda, int pin_scl ) {
  bool success = Wire.begin(pin_sda, pin_scl);
  Wire.setClock( 400000 ); //in ESP8266 library, can't set clock in same call that sets pins
  return success;
}  // end I2CInitialize()

/**************************************************************************/
/*!
    @brief  Read single byte from address and place in destination
    Returns true if successful, false if error
*/
/**************************************************************************/
bool I2CReadByte(byte address, byte reg, byte *destination) {
  return I2CReadBytes(address, reg, destination, 1);
}  // end ReadByte()

/**************************************************************************/
/*!
    @brief  Read num_bytes bytes from address starting at register.
    Assumes device auto-increments the register.
    Bytes read are placed in destination.
    Returns true if successful, false if error.
*/
/**************************************************************************/
bool I2CReadBytes(byte address, byte reg, byte *destination, int num_bytes) {
  if (NULL == destination) {
    return false;
  }
  Wire.beginTransmission(address);
  if (!Wire.write(reg)) {
    Wire.endTransmission(true);
    return false;
  }
  Wire.endTransmission(false);
  if (num_bytes == Wire.requestFrom(address, (uint8_t)num_bytes)) {
    int return_value;
    for (int i=0; i < num_bytes; i++) {
        return_value = Wire.read();
        if (return_value >= 0) {
          destination[i] = (byte)return_value;
        } else {
          return false;
        }
    }//loop through requested number of bytes
    return true;
  }
  return false;

}  // end I2CReadBytes()


/**************************************************************************/
/*!
    @brief  Write single byte to register at address
    Returns true if successful, false if error.
*/
/**************************************************************************/
bool I2CWriteByte(byte address, byte reg, byte value) {
  Wire.beginTransmission(address);
  Wire.write(reg);
  Wire.write(value);
  if (I2C_ERROR_OK == Wire.endTransmission()) {
    return true;
  } else {
    return false;
  }
}  // end I2CWriteByte()

/**************************************************************************/
/*!
    @brief  Write multiple bytes starting at register to I2C address
    Assumes device auto-increments the I2C register being written to.
    Returns true if successful, false if error.
*/
/**************************************************************************/
bool I2CWriteBytes(byte address, byte reg, const byte *value,
               unsigned int num_bytes) {
  Wire.beginTransmission(address);
  Wire.write(reg);
  if (num_bytes != Wire.write(value, num_bytes)) {
    // error queueing up the bytes
    Wire.endTransmission();
    return false;
  }
  if( I2C_ERROR_OK == Wire.endTransmission() ){
    return true;
  }else {
    return false;
  }
} // end I2CWriteBytes()

/*
Call sequence is:
Wire:::endTransmission() -> 
    Wire:::writeTransmission() -> 
        HAL:::i2cWrite() -> 
            HAL:::i2cProcQueue() blocks until queue empty, or bus timeout
*/

//The interface function to write register data from list to a sensor.
int8_t Sensor_I2C_Write_List(registerDeviceInfo_t *devInfo, uint16_t peripheralAddress,
                         const registerwritelist_t *pRegWriteList) {
  // Validate handle
  if (pRegWriteList == NULL) {
    return SENSOR_ERROR_BAD_ADDRESS;
  }

  const registerwritelist_t *pCmd = pRegWriteList;
  // Update register values based on register write list until the next Cmd is
  // the list terminator.
  // original method used Repeated starts, but try individual xactions for simplicity. 
  //   repeatedStart = (pCmd + 1)->writeTo != 0xFFFF;
  // Original method included a mask for the written
  // value, but the mask was always 0x00 (no effect)
  while (pCmd->writeTo != 0xFFFF) {
    // 
    // Set the register based on the values in the register value pair
    // was Register_I2C_Write(pCommDrv, devInfo, peripheralAddress, pCmd->writeTo,
    // pCmd->value, pCmd->mask, repeatedStart);
    if (!I2CWriteByte(peripheralAddress, pCmd->writeTo, pCmd->value)) {
      return SENSOR_ERROR_WRITE;
    }
    ++pCmd;
  };

  return SENSOR_ERROR_NONE;
} // end Sensor_I2C_Write_List()

// Read register data from peripheralAddress, using register
//  location and number of bytes in pReadList.
// Iterate through pReadList until number of bytes requested == 0
// Data is placed sequentially starting at pOutBuffer.
int32_t Sensor_I2C_Read(registerDeviceInfo_t *devInfo,
                        uint16_t peripheralAddress,
                        const registerReadlist_t *pReadList,
                        uint8_t *pOutBuffer) {
//  int32_t status;
  uint8_t *pBuf;

  // Validate handle
  if (pReadList == NULL || pOutBuffer == NULL) {
    return SENSOR_ERROR_BAD_ADDRESS;
  }
  const registerReadlist_t *pCmd = pReadList;

  // Traverse the read list and read the registers one by one unless the
  // register read list numBytes is zero
  for (pBuf = pOutBuffer; pCmd->numBytes != 0; pCmd++) {
    // was Register_I2C_Read(pCommDrv, devInfo, peripheralAddress,
    // pCmd->readFrom, pCmd->numBytes, pBuf);
    if (!I2CReadBytes(peripheralAddress, pCmd->readFrom, pBuf,
                      pCmd->numBytes)) {
      return SENSOR_ERROR_READ;
    }
    pBuf += pCmd->numBytes;
  }
  return SENSOR_ERROR_NONE;
}  // end Sensor_I2C_Read()

int32_t Sensor_I2C_Read_Register(registerDeviceInfo_t *devInfo, 
                          uint16_t peripheralAddress, 
                          uint8_t offset,
                          uint8_t length,
                          uint8_t *pOutBuffer) {
  //TODO - can toss the devInfo parameter, or use it for peripheralAddr
  if(I2CReadBytes((byte)peripheralAddress, (byte)offset, pOutBuffer,
                      (int)length) )
  { return SENSOR_ERROR_NONE;
  } else
  {
    return SENSOR_ERROR_READ;
  }

}//end Sensor_I2C_Read_Register()

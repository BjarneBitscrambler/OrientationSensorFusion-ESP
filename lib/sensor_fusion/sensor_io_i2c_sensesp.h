/*
 * Copyright (c) 2015-2016, Freescale Semiconductor, Inc.
 * Copyright 2016-2017 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 * 
 *  Modified by Bjarne Hansen 2020-10-24 for SensESP environment
 */

/**
 * @file sensor_io_i2c_sensesp.h
 * @brief The sensor_io_i2c_sensesp.h file declares low-level interface functions for reading
 *  and writing sensor registers using I2C.
 */

#ifndef __SENSOR_IO_I2C_SENSESP_H
#define __SENSOR_IO_I2C_SENSESP_H


#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

/*******************************************************************************
 * API
 ******************************************************************************/
/**
\brief Access structure of the I2C Driver. Copied from Driver_I2C.h, and pruned
*/
#define ARM_DRIVER_I2C void     //used to be a struct - now unused

//TODO put these in a class
bool I2CReadByte(uint8_t address, uint8_t reg, uint8_t *destination);
bool I2CReadBytes(uint8_t address, uint8_t reg, uint8_t *destination, int num_bytes);
bool I2CWriteByte(uint8_t address, uint8_t reg, uint8_t value);
bool I2CWriteBytes(uint8_t address, uint8_t reg, const uint8_t *value,
                unsigned int num_bytes);

/*! @brief       Write register data to a sensor

 *  @param[in]   pCommDrv      pointer to the I2C ARM driver to use
 *  @param[in]   devInfo       The I2C device number and idle function.
 *  @param[in]   slaveAddress  the I2C slave address to write to
 *  @param[in]   pRegWriteList a list of one or more register/value pairs to write
 *
 *  @return      returns the execution status of the operation using ::ESensorErrors
 */
int8_t Sensor_I2C_Write_List(ARM_DRIVER_I2C *pCommDrv,
                         registerDeviceInfo_t *devInfo,
                         uint16_t slaveAddress,
                         const registerwritelist_t *pRegWriteList);

/*! @brief       Read register data from a sensor

 *  @param[in]   pCommDrv      pointer to the I2C ARM driver to use
 *  @param[in]   devInfo       The I2C device number and idle function.
 *  @param[in]   slaveAddress  the I2C slave address to read from
 *  @param[in]   pReadList     a list of one or more register addresses and lengths to read
 *  @param[in]   pOutBuffer    a pointer of sufficient size to contain the requested read data
 *
 *
 *  @return      returns the execution status of the operation using ::ESensorErrors
 */
int32_t Sensor_I2C_Read(ARM_DRIVER_I2C *pCommDrv,
                        registerDeviceInfo_t *devInfo,
                        uint16_t slaveAddress,
                        const registerReadlist_t *pReadList,
                        uint8_t *pOutBuffer);

int32_t Sensor_I2C_Read_Register(ARM_DRIVER_I2C *pCommDrv,
                          registerDeviceInfo_t *devInfo,
                          uint16_t peripheralAddress, uint8_t offset,
                          uint8_t length, uint8_t *pOutBuffer);


#ifdef __cplusplus
}
#endif

                          
#endif /* __SENSOR_IO_I2C_SENSESP_H */

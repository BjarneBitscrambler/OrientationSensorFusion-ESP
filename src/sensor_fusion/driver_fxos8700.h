/*
 * Copyright (c) 2015 - 2016, Freescale Semiconductor, Inc.
 * Copyright 2016-2017 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/**
 * @file driver_fxos8700.h
 * @brief Describes the fxos8700 driver interface and structures.
 */

#ifndef DRIVER_FXOS8700_H_
#define DRIVER_FXOS8700_H_


#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#include "driver_sensors_types.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*!
 * @brief This defines the sensor specific information for I2C.
 */
typedef struct
{
    registerDeviceInfo_t deviceInfo; /*!< I2C device context. */
    bool isInitialized;              /*!< whether sensor is intialized or not.*/
    uint16_t slaveAddress;           /*!< slave address.*/
} fxos8700_i2c_sensorhandle_t;

/*! @brief This structure defines the fxos8700 raw data buffer.*/
typedef struct
{
    uint32_t timestamp; /*! The time this sample was recorded.  */
    int16_t accel[3];   /*!< The accel data */
    int16_t mag[3];     /*!< The mag data */
} fxos8700_accelmagdata_t;

/*! @brief This structure defines the fxos8700 raw accel data buffer.*/
typedef struct
{
    uint32_t timestamp; /*! The time this sample was recorded.  */
    int16_t accel[3];   /*!< The accel data */
} fxos8700_acceldata_t;

/*! @def    FXOS8700_SPI_MAX_MSG_SIZE
 *  @brief  The MAX size of SPI message. */
#define FXOS8700_SPI_MAX_MSG_SIZE (64)

/*! @def    FXOS8700_SPI_CMD_LEN
 *  @brief  The size of the Sensor specific SPI Header. */
#define FXOS8700_SPI_CMD_LEN (2)

/*! @def    FXOS8700_SS_ACTIVE_VALUE
 *  @brief  Is the Slave Select Pin Active Low or High. */
#define FXOS8700_SS_ACTIVE_VALUE SPI_SS_ACTIVE_LOW

/*******************************************************************************
 * APIs
 ******************************************************************************/
//TODO - can toss the Idle functions

/*! @brief       The interface function to initialize the sensor.
 *  @details     This function initializes the sensor and sensor handle.
 *  @param[in]   pSensorHandle  handle to the sensor.
 *  @param[in]   index          the I2C device number.
 *  @param[in]   sAddress       slave address of the device on the bus.
 *  @param[in]   whoami         WHO_AM_I value of the device.
 *  @constraints This should be the first API to be called.
 *               Application has to ensure that previous instances of these APIs have exited before invocation.
 *  @reeentrant  No
 *  @return      ::FXOS8700_I2C_Initialize() returns the status .
 */
int32_t FXOS8700_I2C_Initialize(
    fxos8700_i2c_sensorhandle_t *pSensorHandle, uint8_t index, uint16_t sAddress, uint8_t whoAmi);

/*! @brief      :  The interface function to set the I2C Idle Task.
 *  @param[in]  :  fxos8700_i2c_sensorhandle_t *pSensorHandle, handle to the sensor handle.
 *  @param[in]  :  registeridlefunction_t idleTask, function pointer to the function to execute on I2C Idle Time.
 *  @param[in]  :  void *userParam, the pointer to the user idle ftask parameters.
 *  @return        void.
 *  @constraints   This can be called any number of times only after FXOS8700_I2C_Initialize().
 *                 Application has to ensure that previous instances of these APIs have exited before invocation.
 *  @reeentrant    No
 */
void FXOS8700_I2C_SetIdleTask(fxos8700_i2c_sensorhandle_t *pSensorHandle,
                              registeridlefunction_t idleTask,
                              void *userParam);

/*! @brief       The interface function to configure the sensor.
 *  @details     This function configure the sensor with requested ODR, Range and registers in the regsiter pair array.
 *  @param[in]   pSensorHandle      handle to the sensor.
 *  @param[in]   pRegWriteList      pointer to the register list.
 *  @constraints This can be called any number of times only after FXOS8700_I2C_Initialize().
 *               Application has to ensure that previous instances of these APIs have exited before invocation.
 *  @reeentrant  No
 *  @return      ::FXOS8700_I2C_Configure() returns the status .
 */
int32_t FXOS8700_I2C_Configure(fxos8700_i2c_sensorhandle_t *pSensorHandle, const registerwritelist_t *pRegWriteList);

/*! @brief       The interface function to read the sensor data.
 *  @details     This function read the sensor data out from the device and returns raw data in a byte stream.
 *  @param[in]   pSensorHandle  handle to the sensor.
 *  @param[in]   pReadList      pointer to the list of device registers and values to read.
 *  @param[out]  pBuffer        buffer which holds raw sensor data.This buffer may be back to back databuffer based
 *                              command read in the list.
 *  @constraints This can be called any number of times only after FXOS8700_I2C_Initialize().
 *               Application has to ensure that previous instances of these APIs have exited before invocation.
 *  @reeentrant  No
 *  @return      ::FXOS8700_I2C_ReadData() returns the status .
 */
int32_t FXOS8700_I2C_ReadData(fxos8700_i2c_sensorhandle_t *pSensorHandle,
                              const registerReadlist_t *pReadList,
                              uint8_t *pBuffer);

/*! @brief       The interface function to De Initialize sensor..
 *  @details     This function made sensor in a power safe state and de initialize its handle.
 *  @param[in]   pSensorHandle      handle to the sensor.
 *  @constraints This can be called only after FXOS8700_I2C_Initialize().
 *               Application has to ensure that previous instances of these APIs have exited before invocation.
 *  @reeentrant  No
 *  @return      ::FXOS8700_I2C_Deinit() returns the status .
 */
int32_t FXOS8700_I2C_Deinit(fxos8700_i2c_sensorhandle_t *pSensorHandle);

#ifdef __cplusplus
}
#endif

#endif // DRIVER_FXOS8700_H_

/*
 * Copyright (c) 2016, Freescale Semiconductor, Inc.
 * Copyright 2016-2017 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/**
 * @file driver_sensors_types.h
 * @brief The driver_sensors_types.h file contains sensor state structs and error definitions.
 */

#ifndef _DRIVER_SENSORS_TYPES_H
#define _DRIVER_SENSORS_TYPES_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/*******************************************************************************
 * Definitions
 ******************************************************************************/
/* @brief This enum defines Write flag for the Register Write. */
typedef enum EWriteFlags
{
    WRITE_OVERWRITE = 0, /* Overwrite the Register Value.*/
    WRITE_MASK = 1       /* Read and Mask and OR it with Register content.*/
} EWriteFlags_t;

/* @brief This enum defines Sensor State. */
enum ESensorErrors
{
    SENSOR_ERROR_NONE = 0,
    SENSOR_ERROR_INVALID_PARAM,
    SENSOR_ERROR_BAD_ADDRESS,
    SENSOR_ERROR_INIT,
    SENSOR_ERROR_WRITE,
    SENSOR_ERROR_READ,
};

/* The MAXIMUM number of Sensor Registers possible. */
#define SENSOR_MAX_REGISTER_COUNT 128 /* As per 7-Bit address. */

/* Used with the RegisterWriteList types as a list terminator */
#define __END_WRITE_DATA__            \
    {                                 \
        .writeTo = 0xFFFF, .value = 0 \
    }

/* Used with the RegisterReadList types as a list terminator */
#define __END_READ_DATA__                 \
    {                                     \
        .readFrom = 0xFFFF, .numBytes = 0 \
    }

/* Used with the Sensor Command List types as a list terminator */
#define __END_WRITE_CMD__                \
    {                                    \
        .writeTo = 0xFFFF, .numBytes = 0 \
    }

/*******************************************************************************
 * Types
 ******************************************************************************/
/*!
 * @brief This structure defines the Write command List.
 */
typedef struct
{
    uint16_t writeTo; /* Address where the value is writes to.*/
    uint8_t value;    /* value. Note that value should be shifted based on the bit position.*/
    uint8_t mask;     /* mask of the field to be set with given value.*/
} registerwritelist_t;

/*!
 * @brief This structure defines the Read command List.
 */
typedef struct
{
    uint16_t readFrom; /* Address where the value is read from .*/
    uint8_t numBytes;  /* Number of bytes to read.*/
} registerReadlist_t;

/*!
 * @brief This is the register idle function type.
 */
typedef void (*registeridlefunction_t)(void *userParam);

/*!
 * @brief This structure defines the device specific info required by register I/O.
 */
typedef struct
{
    registeridlefunction_t idleFunction;
    void *functionParam;
    uint8_t deviceInstance;
} registerDeviceInfo_t;


#ifdef __cplusplus
}
#endif

#endif //_DRIVER_SENSORS_TYPES_H

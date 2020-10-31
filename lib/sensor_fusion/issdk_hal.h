/*
 * Copyright (c) 2015, Freescale Semiconductor, Inc.
 * Copyright 2016-2017 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause

modified Bjarne Hansen 2020-10-27 for ESP32 environment

 */

/*! \file issdk_hal.h
    \brief Wrapper for Hardware Abstraction Layer (HAL)

*/

#ifndef __ISSDK_HAL_H__
#define __ISSDK_HAL_H__

#ifdef __cplusplus
extern "C" {
#endif

//#include "fsl_i2c_cmsis.h"
//#include "fsl_dspi_cmsis.h"
//#include "fsl_uart_cmsis.h"

//#include "frdm_k22f.h"              //Include appropriate MCU board header file
//#include "frdm_stbc_agm01_shield.h" //Include appropriate sensor shield board header file

// Pin mapping and driver information for default I2C brought to shield
// By default, we use I2C_S1 defined in the frdm_k22f.h file.
// Other options: I2C_S2.
// S1 is on A5:4.  S2 is on D15:14.
//#define I2C_S_SCL_PIN      I2C_S1_SCL_PIN
//#define I2C_S_SDA_PIN      I2C_S1_SDA_PIN
//#define I2C_S_DRIVER       I2C_S1_DRIVER
//#define I2C_S_SIGNAL_EVENT I2C_S1_SIGNAL_EVENT
//#define I2C_S_DEVICE_INDEX I2C_S1_DEVICE_INDEX

#define CORE_SYSTICK_HZ 1000000     //since we know we have 1us resolution available on ESP processors

#ifdef __cplusplus
}
#endif

#endif  // __ISSDK_HAL_H__

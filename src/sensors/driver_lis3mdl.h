/*
 * Copyright (c) 2026, Bjarne Hansen
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/** 
 * @file driver_lis3mdl.h
 * @brief Contains the LIS3MDL Magnetometer sensor register definitions and functions
*/

#ifndef DRIVER_LIS3MDL_H_
#define DRIVER_LIS3MDL_H_

#include "driver_sensors_types.h"

#ifdef __cplusplus
extern "C" {
#endif


int8_t LIS3MDL_Mag_Init(PhysicalSensor *sensor, SensorFusionGlobals *sfg);
int8_t LIS3MDL_Mag_Read(PhysicalSensor *sensor, SensorFusionGlobals *sfg);

/**
    Essential steps for LIS3MDL initialization: 

    WHO_AM_I (0x0F):  Read to verify device (should return 0x3D).
    CTRL_REG1 (0x20): Set Output Data Rate (ODR), XY-axis performance mode, and enable Temp sensor.
    CTRL_REG2 (0x21): Set Full-Scale Range
    CTRL_REG3 (0x22): Set Operating Mode (Continuous-conversion, Single-conversion, or Power-down).
    CTRL_REG4 (0x23): Set Z-axis performance mode and Endianness. 

Initialization Checklist

    Address: Default I2C address is 0x1C (if SA1 pin is low), or 0x1E (if SA1 pin is high).
    Power-Down: Upon reset, the sensor starts in Power-Down mode and must be configured to 
        Continuous or Single mode to start taking readings.

General I2C Operation:
once a slave acknowledge (SAK) has been returned,
an 8-bit subaddress (SUB) is transmitted: the 7 LSb represent the actual register address while the MSb enables
address autoincrement. If the MSb of the SUB field is 1, the SUB (register address) is automatically increased to
allow multiple data read/write. In order to read multiple bytes, it is necessary to assert the most significant bit of the subaddress field. In other
words, SUB(7) must be equal to 1, while SUB(6-0) represents the address of the first register to be read.
*/

/**
 ** @brief LIS3MDL Sensor Register Map.
 */
enum {
     LIS3MDL_WHOAMI             = 0x0F,
     LIS3MDL_CTRL_REG1          = 0x20,
     LIS3MDL_CTRL_REG2          = 0x21,
     LIS3MDL_CTRL_REG3          = 0x22,
     LIS3MDL_CTRL_REG4          = 0x23,
     LIS3MDL_STATUS_REG         = 0x27,
     LIS3MDL_OUT_X_L            = 0x28, //X axis data low byte, two's-complement
     LIS3MDL_OUT_X_H            = 0x29, //X axis data high byte, two's-complement
     LIS3MDL_OUT_Y_L            = 0x2a, //Y axis data low byte, two's-complement
     LIS3MDL_OUT_Y_H            = 0x2b, //Y axis data high byte, two's-complement
     LIS3MDL_OUT_Z_L            = 0x2c, //Z axis data low byte, two's-complement
     LIS3MDL_OUT_Z_H            = 0x2d, //Z axis data high byte, two's-complement
     LIS3MDL_OUT_TEMP_L         = 0x2e, //Temperature sensor data low byte, two's-complement
     LIS3MDL_OUT_TEMP_H         = 0x2f, //Temperature sensor data high byte, two's-complement
     
};

#define LIS3MDL_WHOAMI_RESPONSE (0x3D)

/**
 * @brief   Output data rate (ODR) register settings 
 * Values are for CTRL_REG1 (TEMP_EN,OM1,OM0,DO2,DO1,DO0,FAST_ODR,ST), bits b1-b6 
 */
typedef enum {
    lis3mdl_lpm_0_625   = 0x00, // low power mode at 0.625 Hz
    lis3mdl_lpm_1_25    = 0x04, // low power mode at 1.25 Hz
    lis3mdl_lpm_2_5     = 0x08, // low power mode at 2.5 Hz
    lis3mdl_lpm_5       = 0x0c, // low power mode at 5 Hz
    lis3mdl_lpm_10      = 0x10, // low power mode at 10 Hz
    lis3mdl_lpm_20      = 0x14, // low power mode at 20 Hz
    lis3mdl_lpm_40      = 0x18, // low power mode at 40 Hz
    lis3mdl_lpm_80      = 0x1c, // low power mode at 80 Hz
    lis3mdl_lpm_1000    = 0x02, // low power mode at 1000 Hz
    lis3mdl_mpm_560     = 0x22, // medium performance mode at 560 Hz
    lis3mdl_hpm_300     = 0x42, // high performance mode at 300 Hz
    lis3mdl_uhpm_155    = 0x62, // ultrahigh performance mode at 155 Hz
} lis3mdl_odr_t;

/**
 * @brief   Temperature measuring (TEMP_EN) register settings 
 * Values are for CTRL_REG1 (TEMP_EN,OM1,OM0,DO2,DO1,DO0,FAST_ODR,ST), bit b7 
 */
typedef enum {
    lis3mdl_temp_off   = 0x00, // low power mode at 0.625 Hz
    lis3mdl_temp_on    = 0x80, // low power mode at 1.25 Hz
} lis3mdl_temp_t;

/**
 * @brief   Self-test (ST) register settings 
 * Values are for CTRL_REG1 (TEMP_EN,OM1,OM0,DO2,DO1,DO0,FAST_ODR,ST), bit b0 
 */
typedef enum {
    lis3mdl_selftest_off    = 0x00, // selftest off
    lis3mdl_selftest_on     = 0x01, // selftest on
} lis3mdl_selftest_t;

/**
 * @brief   Full-scale (FS) register settings 
 * Values are for CTRL_REG2 (0,FS1,FS0,0,REBOOT,SOFT_RST,0,0), bits b5,b6 
 */
typedef enum {
    lis3mdl_scale_4G    = 0x00, // 4 Gauss
    lis3mdl_scale_8G    = 0x20, // 8 Gauss
    lis3mdl_scale_12G   = 0x40, // 12 Gauss
    lis3mdl_scale_16G   = 0x60, // 16 Gauss
} lis3mdl_scale_t;

/**
 * @brief   Reboot register setting. Reloads trimming parameters from memory.
 * ST's App Note AN5069 says this takes 20ms, and should be done after a Reset.
 * It is automatically done after power-on.
 * Values are for CTRL_REG2 (0,FS1,FS0,0,REBOOT,SOFT_RST,0,0), bit b3
 */
typedef enum {
    lis3mdl_reboot_off = 0x00,     // default
    lis3mdl_reboot_on  = 0x08,
} lis3mdl_reboot_t;

/**
 * @brief   Reset register setting. Clears Configuration and User registers.
 * ST's App Note AN5069 says this takes 5us, and should be done before a Reset.
 * Values are for CTRL_REG2 (0,FS1,FS0,0,REBOOT,SOFT_RST,0,0), bit b2 
 */
typedef enum {
    lis3mdl_reset_off = 0x00,     // default
    lis3mdl_reset_on  = 0x04,
} lis3mdl_reset_t;

/**
 * @brief   Operating mode register setting. 
 * Selects between single & continuous conversion.
 * Values are for CTRL_REG3 (0,0,LP,0,0,SIM,MD1,MD0), bits b1,b0 
 */
typedef enum {
    lis3mdl_mode_continuous = 0x00,     
    lis3mdl_mode_single     = 0x01,     
    lis3mdl_mode_powerdown  = 0x03,     // default
} lis3mdl_mode_t;

/**
 * @brief   Z-axis performance mode register setting. 
 * Values are for CTRL_REG4 (0,0,0,0,OMZ1,OMZ0,BLE,0), bits b2,b3 
 */
typedef enum {
    lis3mdl_zmode_lowpower  = 0x00,     // default
    lis3mdl_zmode_medium    = 0x04,     
    lis3mdl_zmode_high      = 0x08,     
    lis3mdl_zmode_ultrahigh = 0x0c,     
} lis3mdl_zmode_t;

/**
 * @brief   Endianness register setting. 
 * Values are for CTRL_REG4 (0,0,0,0,OMZ1,OMZ0,BLE,0), bit b1 
 */
typedef enum {
    lis3mdl_ble_lsblow  = 0x00,     // default
    lis3mdl_ble_lsbhigh = 0x02,     
} lis3mdl_ble_t;

/**
 * @brief   Status register bit positions 
 * Values are for STATUS_REG (ZYXOR,ZOR,YOR,XOR,ZYXDA,ZDA,YDA,XDA)
 * The values can be used as a bitmask to isolate a bit. 
 */
typedef enum {
    lis3mdl_status_XDA      = 0x01, //bit set to 1 if new data avail 
    lis3mdl_status_YDA      = 0x02, //bit set to 1 if new data avail
    lis3mdl_status_ZDA      = 0x04, //bit set to 1 if new data avail
    lis3mdl_status_ZYXDA    = 0x08, //bit set to 1 if new data avail
    lis3mdl_status_XOR      = 0x10, //bit set to 0 if no data overrun
    lis3mdl_status_YOR      = 0x20, //bit set to 0 if no data overrun
    lis3mdl_status_ZOR      = 0x40, //bit set to 0 if no data overrun
    lis3mdl_status_ZYXOR    = 0x80, //bit set to 0 if no data overrun
} lis3mdl_status_t;



#ifdef __cplusplus
}
#endif

#endif /* DRIVER_LIS3MDL_H_ */ 
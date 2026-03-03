/*
 * Copyright (c) 2026, Bjarne Hansen
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/**
 * @file driver_LSM6DSOX.h
 * @brief Describes the LSM6DSOX driver interface and structures.
 */

#ifndef DRIVER_LSM6DSOX_H_
#define DRIVER_LSM6DSOX_H_

#include <stdint.h>
#include "driver_sensors_types.h"

#ifdef __cplusplus
extern "C" {
#endif

int8_t LSM6DSOX_All_Init( PhysicalSensor *sensor, SensorFusionGlobals *sfg, bool force );
int8_t LSM6DSOX_Gyro_Init(PhysicalSensor *sensor, SensorFusionGlobals *sfg);
int8_t LSM6DSOX_Gyro_Read(PhysicalSensor *sensor, SensorFusionGlobals *sfg);
int8_t LSM6DSOX_Accel_Init(PhysicalSensor *sensor, SensorFusionGlobals *sfg);
int8_t LSM6DSOX_Accel_Read(PhysicalSensor *sensor, SensorFusionGlobals *sfg);
int8_t LSM6DSOX_Therm_Init(PhysicalSensor *sensor, SensorFusionGlobals *sfg);
int8_t LSM6DSOX_Therm_Read(PhysicalSensor *sensor, SensorFusionGlobals *sfg);

/**
 ** @brief LSM6DSOX Sensor Register Map.
 */
enum {
    LSM6DSOX_WHOAMI     = 0x0F, //(0x6c)
    LSM6DSOX_CTRL1_XL   = 0x10, //(ODR_XL3, ODR_XL2, ODR_XL1, ODR_XL0, FS1_XL, FS0_XL, LPF2_XL_EN, 0)
    LSM6DSOX_CTRL2_G    = 0x11, //(ODR_G3, ODR_G2, ODR_G1, ODR_G0, FS1_G, FS0_G, FS_125, 0)
    LSM6DSOX_CTRL3_C    = 0x12, //(BOOT, BDU, H_LACTIVE, PP_OD, SIM, IF_INC, 0, SW_RESET) default 0x04
    LSM6DSOX_CTRL5_C    = 0x14, //(XL_ULP_EN, ROUNDING1, ROUNDING0, ROUNDING_STATUS, ST1_G, ST0_G, ST1_XL, ST0_XL)
    LSM6DSOX_CTRL6_C    = 0x15, //(TRIG_EN, LVL1_EN, LVL2_EN, XL_HM_MODE, USR_OFF_W, FTYPE_2, FTYPE_1, FTYPE_0)
    LSM6DSOX_OUT_TEMP_L = 0x20, //Temperature output, low byte
    LSM6DSOX_OUT_TEMP_H = 0x21, //Temperature output, high byte, two's complement sign-extended on MSbit
    LSM6DSOX_OUTX_L_G   = 0x22, //Gyro X axis output, low byte
    LSM6DSOX_OUTX_H_G   = 0x23, //Gyro X axis output, high byte, two's complement sign-extended on MSbit
    LSM6DSOX_OUTY_L_G   = 0x24, //Gyro Y axis output, low byte
    LSM6DSOX_OUTY_H_G   = 0x25, //Gyro Y axis output, high byte, two's complement sign-extended on MSbit
    LSM6DSOX_OUTZ_L_G   = 0x26, //Gyro Z axis output, low byte
    LSM6DSOX_OUTZ_H_G   = 0x27, //Gyro Z axis output, high byte, two's complement sign-extended on MSbit
    LSM6DSOX_OUTX_L_A   = 0x28, //Accelerometer X axis output, low byte
    LSM6DSOX_OUTX_H_A   = 0x29, //Accelerometer X axis output, high byte, two's complement sign-extended on MSbit
    LSM6DSOX_OUTY_L_A   = 0x2a, //Accelerometer Y axis output, low byte
    LSM6DSOX_OUTY_H_A   = 0x2b, //Accelerometer Y axis output, high byte, two's complement sign-extended on MSbit
    LSM6DSOX_OUTZ_L_A   = 0x2c, //Accelerometer Z axis output, low byte
    LSM6DSOX_OUTZ_H_A   = 0x2d, //Accelerometer Z axis output, high byte, two's complement sign-extended on MSbit
};

#define LSM6DSOX_WHOAMI_RESPONSE (0x6C)


/**
 * @brief   CTRL1_XL register settings
 * (ODR_XL3, ODR_XL2, ODR_XL1, ODR_XL0, FS1_XL, FS0_XL, LPF2_XL_EN, 0)
 */
typedef enum {
    lsm6dsox_ctrl1xl_odr104hz   = 0x40, //ODR 104Hz, bits [7:4].
    lsm6dsox_ctrl1xl_odr208hz   = 0x50, //ODR 208Hz, bits [7:4].
    lsm6dsox_ctrl1xl_fs2g       = 0x00, //Full scale +/- 2g
    lsm6dsox_ctrl1xl_fs4g       = 0x08, //Full scale +/- 4g
    lsm6dsox_ctrl1xl_fs8g       = 0x0c, //Full scale +/- 8g
    lsm6dsox_ctrl1xl_lpf2xl     = 0x02, //High-resolution selection: LPF2 filtering stage selected when bit set = 1
} lsm6dsox_ctrl1xl_t;

/**
 * @brief   CTRL2_G register settings
 * (ODR_G3, ODR_G2, ODR_G1, ODR_G0, FS1_G, FS0_G, FS_125, 0)
 */
typedef enum {
    lsm6dsox_ctrl2g_odr104hz   = 0x40, //ODR 104Hz, bits [7:4].
    lsm6dsox_ctrl2g_fs250       = 0x00, //Full scale +/- 250dps
    lsm6dsox_ctrl2g_fs500      = 0x04, //Full scale +/- 500 dps
    lsm6dsox_ctrl2g_fs1000      = 0x08, //Full scale +/- 1000 dps
    lsm6dsox_ctrl2g_fs2000      = 0x0c, //Full scale +/-2000 dps
    lsm6dsox_ctrl2g_fs125     = 0x02, //High-resolution selection: full scale +/-125dps when bit set = 1
} lsm6dsox_ctrl2g_t;

/**
 * @brief   CTRL3_C register settings
 * (BOOT,BDU,H_LACTIVE,PP_OD,SIM,IF_INC,0,SW_RESET)
 */
typedef enum {
    lsm6dsox_ctrl3c_reboot  = 0x80, //Reboot memory content. Bit automatically clears after boot.
    lsm6dsox_ctrl3c_bdu     = 0x40, //Output data MSB and LSB update & read synchronously
    lsm6dsox_ctrl3c_ifinc   = 0x04, //auto-increment register address for multi-byte reads. Default = 1 = increment.
    lsm6dsox_ctrl3c_swreset = 0x01, //Software reset. Bit automatically clears after reset.
} lsm6dsox_ctrl3c_t;

/**
 * @brief   CTRL5_C register settings
 * (XL_ULP_EN, ROUNDING1, ROUNDING0, ROUNDING_STATUS, ST1_G, ST0_G, ST1_XL, ST0_XL)
 */
typedef enum {
    lsm6dsox_ctrl5c_xlulp  = 0x80, //Accelerometer Ultralow power enable when = 1. Default 0
    lsm6dsox_ctrl5c_wrapa  = 0x20, //Circular read mode (wraparound) from accel registers. Default 00 (no wrap)
    lsm6dsox_ctrl5c_wrapg  = 0x40, //Circular read mode (wraparound) from gyro registers. Default 00 (no wrap)
    lsm6dsox_ctrl5c_wrapall= 0x60, //Circular read mode (wraparound) from all output registers. Default 00 (no wrap)
} lsm6dsox_ctrl5c_t;

/**
 * @brief   CTRL6_C register settings
 * (TRIG_EN, LVL1_EN, LVL2_EN, XL_HM_MODE, USR_OFF_W, FTYPE_2, FTYPE_1, FTYPE_0)
 */
typedef enum {
    lsm6dsox_ctrl6c_xlhmmode  = 0x10, //Accelerometer High-Performance enable when = 0. Default = 0
} lsm6dsox_ctrl6c_t;





#ifdef __cplusplus
}
#endif

#endif // DRIVER_LSM6DSOX_H_

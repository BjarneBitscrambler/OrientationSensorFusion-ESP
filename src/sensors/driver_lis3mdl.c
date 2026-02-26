/*
 * Copyright (c) 2026 Bjarne Hansen
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/** @file driver_lis3mdl.c
    @brief Commands to interact with LIS3MDL magnetometer IC (e.g. read Device ID, Initialize, Read Data). 
    Actual I2C interface functions are found in hal_i2c files.
*/

#include "../fusion/hal_i2c.h"         // I2C interface methods
#include "../fusion/sensor_fusion.h"   // Sensor fusion structures and types
#include "driver_lis3mdl.h"            // LIS3MDL IC data and control registers

/** @brief Create list of initialization registers and the bytes to write
 *  Each entry in a RegisterWriteList is composed of: register address, 
 *  value to write, bit-mask to apply to write (0 enables).
 *  Assumes LIS3MDL has already been reset, and is in standby mode.
 */
const registerwritelist_t   LIS3MDL_Initialization[] =
{
    // CTRL_REG1 (0x20): Set Output Data Rate (ODR), XY-axis performance mode, and enable Temp sensor.
    { .writeTo =    LIS3MDL_CTRL_REG1, 
      .value =      lis3mdl_uhpm_155 | lis3mdl_temp_on | lis3mdl_selftest_off,
      .mask =       0x00 
    }, 

    // CTRL_REG2 (0x21): Set Full-Scale Range
    {   LIS3MDL_CTRL_REG2, 
        lis3mdl_scale_4G, 
        0x00 
    },
    #define LIS3MDL_COUNTSPERGAUSS  6842    //based on choice of full-scale = 4 Gauss
    #define UT_PER_GAUSS            100     //conversion factor, since rest of code works in T and uT

    // CTRL_REG3 (0x22): Set Operating Mode (Continuous-conversion, Single-conversion, or Power-down).
    {   LIS3MDL_CTRL_REG3, 
        lis3mdl_mode_continuous, 
        0x00 
    },   

    // CTRL_REG4 (0x23): Set Z-axis performance mode and Endianness. 
    {   LIS3MDL_CTRL_REG4, 
        lis3mdl_zmode_ultrahigh | lis3mdl_ble_lsblow, 
        0x00 
    },

   // CTRL_REG5 (0x24): Set Block Data Update mode so LSb and MSb updated synchronously. 
    {   LIS3MDL_CTRL_REG5, 
        lis3mdl_ctrlreg5_bdu, 
        0x00 
    },

    __END_WRITE_DATA__
};

/** @brief Check for presence of LIS3MDL and configure its registers
 * 
 *  Start it capturing data in continuous mode.
 *  @return Status as enum ESensorErrors
 */
int8_t LIS3MDL_Mag_Init(struct PhysicalSensor *sensor, SensorFusionGlobals *sfg) 
{
    int32_t status;
    uint8_t reg;

    status = Sensor_I2C_Read_Register(&sensor->deviceInfo, sensor->addr, LIS3MDL_WHOAMI, 1, &reg);

    if (status==SENSOR_ERROR_NONE) 
    {  sfg->Mag.iWhoAmI = reg;
       if (reg != LIS3MDL_WHOAMI_RESPONSE) 
       {  return SENSOR_ERROR_INIT;  // The whoAmI did not match
       }
    } else 
    {  // whoAmI will retain default value of zero
       // return with error
       return status;
    }

    //reset, then reboot the LIS3MDL. This sets registers to default and reloads
    //trimming values from memory. See ST's app note AN5069.
    if( !I2CWriteByte(sensor->addr, LIS3MDL_CTRL_REG2, lis3mdl_reset_on) )
    {  return SENSOR_ERROR_INIT;
    }
    delay(1);   //wait one ms (alternative is a blocking call to wait 5us)
    if( !I2CReadByte(sensor->addr, LIS3MDL_CTRL_REG2, &reg) || (reg != 0x00) )
    {   return SENSOR_ERROR_INIT;
    }
    if( !I2CWriteByte(sensor->addr, LIS3MDL_CTRL_REG2, lis3mdl_reboot_on) )
    {  return SENSOR_ERROR_INIT;
    }
    delay(22);   //wait 22 ms (AN5069 says it takes 20ms)      

    // Configure and start the LIS3MDL sensor.  This does multiple register writes
    status = Sensor_I2C_Write_List(&sensor->deviceInfo, sensor->addr, LIS3MDL_Initialization );

    sensor->isInitialized = F_USING_MAG;
    sfg->Mag.isEnabled = true;
    sfg->Mag.iCountsPeruT = (int) (LIS3MDL_COUNTSPERGAUSS / UT_PER_GAUSS);
    sfg->Mag.fCountsPeruT = (float) (LIS3MDL_COUNTSPERGAUSS / UT_PER_GAUSS);
    sfg->Mag.fuTPerCount = UT_PER_GAUSS / LIS3MDL_COUNTSPERGAUSS;

    return (status);
} // end LIS3MDL_Mag_Init()


 /** @brief Fetch magnetometer data from LIS3MDL
  *  Magnetometer is set up for a data rate of 155 samples/s.
  *  When fusing at 40 Hz, we will have data overruns, but that
  *  is OK (TBC).
  *  @return Status as enum ESensorErrors
 */
int8_t LIS3MDL_Mag_Read(PhysicalSensor *sensor, SensorFusionGlobals *sfg)
{
    uint8_t                     I2C_Buffer[6];  // I2C read buffer
    int32_t                     status;         // I2C transaction status
    int16_t                     sample[3];
    byte                        reg;

    if(!(sensor->isInitialized & F_USING_MAG))
    {   return SENSOR_ERROR_INIT;
    }

    // read the six sequential magnetometer output bytes.
    // After the START condition (ST) a slave address is sent, once a slave acknowledge (SAK) has been returned,
    //an 8-bit subaddress (SUB) is transmitted: the 7 LSb represent the actual register address while the MSb enables
    //address autoincrement. If the MSb of the SUB field is 1, the SUB (register address) is automatically increased to
    //allow multiple data read/write. So we set the MSbit of the Register to ask for autoincrement.
    reg = LIS3MDL_OUT_X_L | 0x80;
    if( ! I2CReadBytes(sensor->addr, reg, I2C_Buffer, 6) )
    {   status = SENSOR_ERROR_READ;
    }else
    {   //Place the 6 bytes read into the magnetometer structure
        //Data were read into the buffer in order: X_LSB, X_MSB, Y_LSB, Y_MSB, Z_LSB, Z_MSB
        //Data in two's-complement 2-byte integers.
        sample[CHX] = (I2C_Buffer[1] << 8) | I2C_Buffer[0];
        sample[CHY] = (I2C_Buffer[3] << 8) | I2C_Buffer[2];
        sample[CHZ] = (I2C_Buffer[5] << 8) | I2C_Buffer[4];
        conditionSample(sample);  // truncate negative values to -32767
        addToFifo((union FifoSensor*) &(sfg->Mag), MAG_FIFO_SIZE, sample);
        status = SENSOR_ERROR_NONE;
    }
        
    return status;
}//end LIS3MDL_Mag_Read()


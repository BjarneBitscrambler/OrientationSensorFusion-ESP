/*
 * Copyright (c) 2026 Bjarne Hansen
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/*! \file driver_lsm6dsox.c
    \brief Functions to interact with LSM6DSOX accelerometer + gyroscope + thermometer
    (e.g. read Device ID, Initialize, Read Data) 
    Actual I2C interface functions are in hal_i2c files.
*/

#include <esp_log.h>

static const char* TAG = "driver_lms6dsox";

#include "../fusion/hal_i2c.h"          // I2C interface methods
#include "../fusion/sensor_fusion.h"    // Sensor fusion structures and types
#include "driver_lsm6dsox.h"            // LSM6DSOX hardware interface

int8_t LSM6DSOX_Gyro_Read(PhysicalSensor *sensor, SensorFusionGlobals *sfg);
int8_t LSM6DSOX_Accel_Read(PhysicalSensor *sensor, SensorFusionGlobals *sfg);
int8_t LSM6DSOX_Therm_Read(PhysicalSensor *sensor, SensorFusionGlobals *sfg);

#define LSM6DSOX_BOOT_TIME_MS   11  //ms. Spec sheet says 10ms - allow a bit longer.
#define LSM6DSOX_RESET_TIME_MS   1  //ms Spec sheet says 50us. 1 ms is min with the delay() call.

#define LSM6DSOX_MILLIGPERCOUNT   0.122  //assumes +/-4 g range on accelerometer
#define LSM6DSOX_MILLIDPSPERCOUNT 8.75   // +/-250dps range. Change if LSM6DSOX_Initialization[] changes.

#define LSM6DSOX_COUNTSPERDEGREEC 256   //conversion from counts to degrees
#define LSM6DSOX_DEGREECOFFSET     25   //add to the scaled value to get degrees C

/** Sensitivities for the LSM6DSOX, from ST's file lsm6dsox_reg.c (GitHub)
*/
#define lsm6dsox_from_fs2_to_mg 0.061
#define lsm6dsox_from_fs4_to_mg 0.122
#define lsm6dsox_from_fs8_to_mg 0.244
#define lsm6dsox_from_fs16_to_mg 0.488
#define lsm6dsox_from_fs125_to_mdps 4.375
#define lsm6dsox_from_fs500_to_mdps 17.50
#define lsm6dsox_from_fs250_to_mdps 8.750
#define lsm6dsox_from_fs1000_to_mdps 35.0
#define lsm6dsox_from_fs2000_to_mdps 70.0
#define lsm6dsox_from_lsb_to_celsius (1/256)    //and add 25C to result.

const registerwritelist_t   LSM6DSOX_Initialization[] =
{
    //Set Accelerometer Output Data Rate (ODR) and Full-scale
    { .writeTo =    LSM6DSOX_CTRL1_XL, 
      .value =      lsm6dsox_ctrl1xl_odr104hz | lsm6dsox_ctrl1xl_fs4g | lsm6dsox_ctrl1xl_lpf2xl,
      .mask =       0x00 
    },     
    //Set Gyro Output Data Rate (ODR) and Full-scale
    { .writeTo =    LSM6DSOX_CTRL2_G, 
      .value =      lsm6dsox_ctrl2g_odr104hz | lsm6dsox_ctrl2g_fs250,
      .mask =       0x00 
    }, 
    //Set Block Data Update
    { .writeTo =    LSM6DSOX_CTRL3_C, 
      .value =      lsm6dsox_ctrl3c_bdu,
      .mask =       0x00 
    }, 
    //Enable wraparound when reading output data registers
    { .writeTo =    LSM6DSOX_CTRL5_C, 
      .value =      lsm6dsox_ctrl5c_wrapall,
      .mask =       0x00 
    }, 
    __END_WRITE_DATA__
};

/** Several different sensors reside on the same physical IC.  Each sensor has its own
 * Init function, but all call LSM6DSOX_All_Init() to perform the configuration.
 * If LSM6DSOX_All_Init() has already run, it merely checks that the IC is still responding
 * to a WhoAmI request, and returns. If it has not run previously, it performs a full init.
 */
int8_t LSM6DSOX_Gyro_Init(PhysicalSensor *sensor, SensorFusionGlobals *sfg)
{
    int32_t status;

    status = LSM6DSOX_All_Init( sensor, sfg, false );
    if( status == SENSOR_ERROR_NONE)
    {   sensor->isInitialized = F_USING_GYRO;
        sfg->Gyro.fDegPerSecPerCount = (float) LSM6DSOX_MILLIDPSPERCOUNT / 1000.0;
        sfg->Gyro.iCountsPerDegPerSec = (int) (1000.0 / LSM6DSOX_MILLIDPSPERCOUNT);
        sfg->Gyro.iWhoAmI = LSM6DSOX_WHOAMI_RESPONSE;
        sfg->Gyro.iFIFOCount=0;
        sfg->Gyro.isEnabled = true;
    }
    ESP_LOGI( TAG, "Ran Gyro_Init()" );
    return status;
}//end LSM6DSOX_Gyro_Init()

/** Several different sensors reside on the same physical IC.  Each sensor has its own
 * Init function, but all call LSM6DSOX_All_Init() to perform the configuration.
 * If LSM6DSOX_All_Init() has already run, it merely checks that the IC is still responding
 * to a WhoAmI request, and returns. If it has not run previously, it performs a full init.
 */
int8_t LSM6DSOX_Accel_Init(PhysicalSensor *sensor, SensorFusionGlobals *sfg)
{
   int32_t status;

    status = LSM6DSOX_All_Init( sensor, sfg, false );
    if( status == SENSOR_ERROR_NONE)
    {   sensor->isInitialized = F_USING_ACCEL;
        sfg->Accel.iCountsPerg = (int) (1000.0 / LSM6DSOX_MILLIGPERCOUNT);
        sfg->Accel.fgPerCount = (float) (LSM6DSOX_MILLIGPERCOUNT / 1000.0);
        sfg->Accel.iWhoAmI = LSM6DSOX_WHOAMI_RESPONSE;
        sfg->Accel.iFIFOCount=0;
        sfg->Accel.isEnabled = true;
    }
    ESP_LOGI( TAG, "Ran Accel_Init()" );
    return status;

}//end LSM6DSOX_Accel_Init()

/** Several different sensors reside on the same physical IC.  Each sensor has its own
 * Init function, but all call LSM6DSOX_All_Init() to perform the configuration.
 * If LSM6DSOX_All_Init() has already run, it merely checks that the IC is still responding
 * to a WhoAmI request, and returns. If it has not run previously, it performs a full init.
 */
int8_t LSM6DSOX_Therm_Init(PhysicalSensor *sensor, SensorFusionGlobals *sfg)
{
   int32_t status;

    status = LSM6DSOX_All_Init( sensor, sfg, false );
    if( status == SENSOR_ERROR_NONE)
    {   sensor->isInitialized = F_USING_TEMPERATURE;
    }
    ESP_LOGI( TAG, "Ran Therm_Init()" );
    return status;

}//end LSM6DSOX_Therm_Init()


/**
 * @brief   Resets the registers of the LSM6DSOX and loads internal trim values.
 * Called by all of the Gyro, Accel, and Therm_Init functions, but only executes
 * the first time, or if called with parameter force = true.
 * @param force. Boolean which when true, forces All_Init to run even if it has
 * done so previously. This allows higher level logic to reset the device if
 * needed.
 * @return Status as enum ESensorErrors
 */
int8_t LSM6DSOX_All_Init( PhysicalSensor *sensor, SensorFusionGlobals *sfg, bool force )
{   static bool firstTimeRun = true;
    int32_t status;
    uint8_t reg;

    if( firstTimeRun || force )
    {   delay(LSM6DSOX_BOOT_TIME_MS); //How long device takes to power-up.  Unlikely that ESP32 is ready before LSM6DSOX
        //Check device ID
        status = Sensor_I2C_Read_Register(&sensor->deviceInfo, sensor->addr, LSM6DSOX_WHOAMI, 1, &reg);
        if (status==SENSOR_ERROR_NONE) 
        {   if (reg != LSM6DSOX_WHOAMI_RESPONSE) 
            {  return SENSOR_ERROR_INIT;  // The whoAmI did not match
            }
        }else 
        {  // whoAmI will retain default value of zero
            // return with error
            return status;
        }

        //reboot then rest the LSM6DSOX. This sets registers to default and reloads
        //trimming values from memory. See ST's app note AN5272
        if( !I2CWriteByte(sensor->addr, LSM6DSOX_CTRL3_C, lsm6dsox_ctrl3c_reboot) )
        {  return SENSOR_ERROR_INIT;
        }
        delay(LSM6DSOX_BOOT_TIME_MS);   //wait >10 ms per ST's AppNote AN5272
        if( !I2CWriteByte(sensor->addr, LSM6DSOX_CTRL3_C, lsm6dsox_ctrl3c_swreset) )
        {  return SENSOR_ERROR_INIT;
        }
        delay(LSM6DSOX_RESET_TIME_MS); //register reset takes 50us per AppNote AN5272. Reset bit should == 0 when complete.
        if( !I2CReadByte(sensor->addr, LSM6DSOX_CTRL3_C, &reg) || ((reg & lsm6dsox_ctrl3c_swreset) != 0x00) )
        {   return SENSOR_ERROR_INIT;
        }
        status = Sensor_I2C_Write_List(&sensor->deviceInfo, sensor->addr, LSM6DSOX_Initialization );
        firstTimeRun = false;
        ESP_LOGI( TAG, "Ran LSM6DSOC_All_Init()" );
    }
    return (status);
}//end LSM6DSOX_All_Init()


int8_t LSM6DSOX_Gyro_Read(PhysicalSensor *sensor, SensorFusionGlobals *sfg)
{
    uint8_t     I2C_Buffer[6];          // I2C read buffer
    int8_t      j;                      // scratch
    uint8_t     fifo_packet_count = 1;  //won't use hardware FIFO
    int16_t     sample[3];

    if(!(sensor->isInitialized & F_USING_GYRO)) 
    {   return SENSOR_ERROR_INIT;
    }
    fifo_packet_count = 1;
    // Following is not needed, since we aren't checking for data ready.
    // Return if there are no measurements in the sensor FIFO.
    // this will only occur when the calling frequency equals or exceeds
    // ACCEL_ODR_HZ
    if (fifo_packet_count == 0) 
    {   return (SENSOR_ERROR_READ);
    }
    if( !I2CReadBytes(sensor->addr, LSM6DSOX_OUTX_L_G, I2C_Buffer, 6) ) 
    {   return SENSOR_ERROR_READ;
    }
    // place the measurements read into the accelerometer buffer structure 
    sample[CHX] = (I2C_Buffer[1] << 8) | (I2C_Buffer[0]); 
    sample[CHY] = (I2C_Buffer[3] << 8) | (I2C_Buffer[2]); 
    sample[CHZ] = (I2C_Buffer[5] << 8) | (I2C_Buffer[4]);
    conditionSample(sample);  //truncate negative values to -32767
    // place the 6 bytes read into the 16 bit accelerometer structure 
    addToFifo((union FifoSensor*) &(sfg->Gyro), GYRO_FIFO_SIZE, sample);

    return SENSOR_ERROR_NONE;
}//end LSM6DSOX_Gyro_Read()

int8_t LSM6DSOX_Accel_Read(PhysicalSensor *sensor, SensorFusionGlobals *sfg)
{
    uint8_t     I2C_Buffer[6];          // I2C read buffer
    int8_t      j;                      // scratch
    uint8_t     fifo_packet_count = 1;  //won't use hardware FIFO
    int16_t     sample[3];

    if(!(sensor->isInitialized & F_USING_ACCEL)) 
    {   return SENSOR_ERROR_INIT;
    }
    fifo_packet_count = 1;
    // Following is not needed, since we aren't checking for data ready.
    // Return if there are no measurements in the sensor FIFO.
    // this will only occur when the calling frequency equals or exceeds
    // ACCEL_ODR_HZ
    if (fifo_packet_count == 0) 
    {   return (SENSOR_ERROR_READ);
    }
    if( !I2CReadBytes(sensor->addr, LSM6DSOX_OUTX_L_A, I2C_Buffer, 6) ) 
    {   return SENSOR_ERROR_READ;
    }
    // place the measurements read into the accelerometer buffer structure 
    sample[CHX] = (I2C_Buffer[1] << 8) | (I2C_Buffer[0]); 
    sample[CHY] = (I2C_Buffer[3] << 8) | (I2C_Buffer[2]); 
    sample[CHZ] = (I2C_Buffer[5] << 8) | (I2C_Buffer[4]);
    conditionSample(sample);  //truncate negative values to -32767
    // place the 6 bytes read into the 16 bit accelerometer structure 
    addToFifo((union FifoSensor*) &(sfg->Accel), ACCEL_FIFO_SIZE, sample);

    return SENSOR_ERROR_NONE;
}//end LSM6DSOX_Accel_Read()

int8_t LSM6DSOX_Therm_Read(PhysicalSensor *sensor, SensorFusionGlobals *sfg)
{
    uint8_t                     I2C_Buffer[2];  // I2C read buffer
    int16_t                     sample;
    static int16_t loops = 0;

    if(!(sensor->isInitialized & F_USING_TEMPERATURE)) 
    {   return SENSOR_ERROR_INIT;
    }
    if( !I2CReadBytes(sensor->addr, LSM6DSOX_OUT_TEMP_L, I2C_Buffer, 2) ) 
    {   return SENSOR_ERROR_READ;
    }
    sample = (I2C_Buffer[1] << 8) | (I2C_Buffer[0]);
    //convert raw reading to Celcius
    sfg->Temp.temperatureC = (float)sample / (float)LSM6DSOX_COUNTSPERDEGREEC + (float)LSM6DSOX_DEGREECOFFSET;

    loops++;
    if( loops % 40 == 0)
    {   ESP_LOGI( "driver_lsm6dsox.h", 
            "Temperature: %d Lowbyte: 0x%x HighByte: 0x%x",
            sfg->Temp.temperatureC, I2C_Buffer[0], I2C_Buffer[1]
        );
    }

    return SENSOR_ERROR_NONE;
}//end LSM6DSOX_Therm_Read()




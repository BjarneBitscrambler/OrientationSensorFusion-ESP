/*
 * Copyright (c) 2015, Freescale Semiconductor, Inc.
 * Copyright 2016-2017 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/*! \file driver_FXAS21002.c
    \brief Provides init() and read() functions for the FXAS21002 gyroscope
*/

#include "sensor_fusion.h"      // Sensor fusion structures and types
#include "driver_fxas21002.h"   // Definitions for FXAS21002 interface
#include "hal_i2c.h"            //I2C interface methods

// Includes support for pre-production FXAS21000 registers and constants which are not supported via IS-SDK
#define FXAS21000_STATUS                0x00
#define FXAS21000_F_STATUS              0x08
#define FXAS21000_F_SETUP               0x09
#define FXAS21000_WHO_AM_I              0x0C
#define FXAS21000_CTRL_REG0             0x0D
#define FXAS21000_CTRL_REG1             0x13
#define FXAS21000_CTRL_REG2             0x14
#define FXAS21000_WHO_AM_I_VALUE        0xD1    // engineering and production
#define FXAS21000_COUNTSPERDEGPERSEC    20      // 1600dps range
#define FXAS21002_COUNTSPERDEGPERSEC    16      // for 2000dps=32000 counts

#if F_USING_GYRO

// Command definition to read the WHO_AM_I value.
const registerReadlist_t    FXAS21002_WHO_AM_I_READ[] =
{
    { .readFrom = FXAS21002_WHO_AM_I, .numBytes = 1 }, __END_READ_DATA__
};

// Command definition to read the number of entries in the gyro status register.
const registerReadlist_t    FXAS21002_F_STATUS_READ[] =
{
    { .readFrom = FXAS21002_STATUS, .numBytes = 1 }, __END_READ_DATA__
};

// Command definition to read the gyro FIFO.
registerReadlist_t          FXAS21002_DATA_READ[] =
{
    { .readFrom = FXAS21002_OUT_X_MSB, .numBytes = 6 }, __END_READ_DATA__
};

// Each entry in a RegisterWriteList is composed of: register address, value to write, bit-mask to apply to write (0 enables)
const registerwritelist_t   FXAS21000_INITIALIZATION[] =
{
    // write 0000 0000 = 0x00 to CTRL_REG1 to place FXOS21000 in Standby
    // [7]: ZR_cond=0
    // [6]: RST=0
    // [5]: ST=0 self test disabled
    // [4-2]: DR[2-0]=000 for 200Hz ODR
    // [1]: Active=0 for Standby mode
    // [0]: Ready=0 but irrelevant since Active bit over-rides
    { FXAS21000_CTRL_REG1, 0x00, 0x00 },        // write 0100 0000 = 0x40 to F_SETUP to enable FIFO in continuous mode

    // [7-6]: F_MODE[1-0]=01 for FIFO continuous mode
    // [5-0]: F_WMRK[5-0]=000000 for no FIFO watermark
    { FXAS21000_F_SETUP, 0x40, 0x00 },

    // write 0000 0000 = 0x00 to CTRL_REG0 to configure range
    // [7-6]: unused=00
    // [5]: SPIW=0 4 wire SPI (irrelevant)
    // [4-3]: SEL[1-0]=00 for HPF cutoff but disabled in HPF_EN
    // [2]: HPF_EN=0 disable HPF
    // [1-0]: FS[1-0]=00 for 1600dps
    { FXAS21000_CTRL_REG0, 0x00, 0x00 },

    // write 000X XX10 to CTRL_REG1 to configure ODR and enter Active mode
    // [7]: Reserved=0
    // [6]: RST=0 for no reset
    // [5]: ST=0 self test disabled
    // [4-2]: DR[2-0]=111 also for 12.5Hz ODR giving 0x1E
    // [4-2]: DR[2-0]=110 for 12.5Hz ODR giving 0x1A
    // [4-2]: DR[2-0]=101 for 25Hz ODR giving 0x16
    // [4-2]: DR[2-0]=100 for 50Hz ODR giving 0x12
    // [4-2]: DR[2-0]=011 for 100Hz ODR giving 0x0E
    // [4-2]: DR[2-0]=010 for 200Hz ODR giving 0x0A
    // [4-2]: DR[2-0]=001 for 400Hz ODR giving 0x06
    // [4-2]: DR[2-0]=000 for 800Hz ODR giving 0x02
    // [1]: Active=1 for Active mode
    // [0]: Ready=0 but irrelevant since Active bit over-rides
#if (GYRO_ODR_HZ <= 1)                      // select 1.5625Hz ODR
    { FXAS21000_CTRL_REG1, 0x1E, 0x00 },
#elif (GYRO_ODR_HZ <= 3)                    // select 3.125Hz ODR
    { FXAS21000_CTRL_REG1, 0x1A, 0x00 },
#elif (GYRO_ODR_HZ <= 6)                    // select 6.25Hz ODR
    { FXAS21000_CTRL_REG1, 0x16, 0x00 },
#elif (GYRO_ODR_HZ <= 12)                   // select 12.5Hz ODR
    { FXAS21000_CTRL_REG1, 0x12, 0x00 },
#elif (GYRO_ODR_HZ <= 25)                   // select 25.0Hz ODR
    { FXAS21000_CTRL_REG1, 0x0E, 0x00 },
#elif (GYRO_ODR_HZ <= 50)                   // select 50.0Hz ODR
    { FXAS21000_CTRL_REG1, 0x0A, 0x00 },
#elif (GYRO_ODR_HZ <= 100)                  // select 100.0Hz ODR
    { FXAS21000_CTRL_REG1, 0x06, 0x00 },
#else // select 200Hz ODR
    { FXAS21000_CTRL_REG1, 0x02, 0x00 },
#endif
    __END_WRITE_DATA__
};

// Each entry in a RegisterWriteList is composed of: register address, value to write, bit-mask to apply to write (0 enables)
const registerwritelist_t   FXAS21002_INITIALIZATION[] =
{
    // write 0000 0000 = 0x00 to CTRL_REG1 to place FXOS21000 in Standby
    // [7]: ZR_cond=0
    // [6]: RST=0
    // [5]: ST=0 self test disabled
    // [4-2]: DR[2-0]=000 for 200Hz ODR
    // [1]: Active=0 for Standby mode
    // [0]: Ready=0 but irrelevant since Active bit over-rides
    { FXAS21002_CTRL_REG1, 0x00, 0x00 },

    // [7-6]: F_MODE[1-0]=01 for FIFO continuous mode
    // [5-0]: F_WMRK[5-0]=000000 for no FIFO watermark
    { FXAS21002_F_SETUP, 0x40, 0x00 },

    // write 0000 0000 = 0x00 to CTRL_REG0 to configure range and LPF
    // [7-6]: BW[1-0]=00 for least aggressive LPF (0.32 * ODR cutoff for all ODR ie 64Hz cutoff at 200Hz ODR)
    // [5]: SPIW=0 4 wire SPI (irrelevant)
    // [4-3]: SEL[1-0]=00 for HPF cutoff but disabled in HPF_EN
    // [2]: HPF_EN=0 to disable HPF
    // [1-0]: FS[1-0]=00 for 2000dps
    { FXAS21002_CTRL_REG0, 0x00, 0x00 },

    // write 0000 1000 = 0x08 to CTRL_REG3 to set FIFO address wraparound on read
    // [7-4]: Reserved=0000
    // [3]: WRAPTOONE=1 to permit burst FIFO read with address wrapround to OUT_X_MSB
    // [2]: EXTCTRLEN=0 for default INT2 configuration as output
    // [1]: Reserved=0
    // [0]: FS_DOUBLE=0 for normal 2000dps range
    { FXAS21002_CTRL_REG3, 0x08, 0x00 },

    // write 000X XX10 to CTRL_REG1 to configure ODR and enter Active mode
    // [7]: Reserved=0
    // [6]: RST=0 for no reset
    // [5]: ST=0 self test disabled
    // [4-2]: DR[2-0]=111 also for 12.5Hz ODR giving 0x1E
    // [4-2]: DR[2-0]=110 for 12.5Hz ODR giving 0x1A
    // [4-2]: DR[2-0]=101 for 25Hz ODR giving 0x16
    // [4-2]: DR[2-0]=100 for 50Hz ODR giving 0x12
    // [4-2]: DR[2-0]=011 for 100Hz ODR giving 0x0E
    // [4-2]: DR[2-0]=010 for 200Hz ODR giving 0x0A
    // [4-2]: DR[2-0]=001 for 400Hz ODR giving 0x06
    // [4-2]: DR[2-0]=000 for 800Hz ODR giving 0x02
    // [1]: Active=1 for Active mode
    // [0]: Ready=0 but irrelevant since Active bit over-rides
    // These values are different than the FXAS21000 values
#if (GYRO_ODR_HZ <= 12)                     // select 12.5Hz ODR
    { FXAS21002_CTRL_REG1, 0x1A, 0x00 },
#elif (GYRO_ODR_HZ <= 25)                   // select 25Hz ODR
    { FXAS21002_CTRL_REG1, 0x16, 0x00 },
#elif (GYRO_ODR_HZ <= 50)                   // select 50Hz ODR
    { FXAS21002_CTRL_REG1, 0x12, 0x00 },
#elif (GYRO_ODR_HZ <= 100)                  // select 100Hz ODR
    { FXAS21002_CTRL_REG1, 0x0E, 0x00 },
#elif (GYRO_ODR_HZ <= 200)                  // select 200Hz ODR
    { FXAS21002_CTRL_REG1, 0x0A, 0x00 },
#elif (GYRO_ODR_HZ <= 400)                  // select 400Hz ODR
    { FXAS21002_CTRL_REG1, 0x06, 0x00 },
#else // select 800Hz ODR
    { FXAS21002_CTRL_REG1, 0x02, 0x00 },
#endif
    __END_WRITE_DATA__
};

// All sensor drivers and initialization functions have the same prototype
// sensor = pointer to linked list element used by the sensor fusion subsystem to specify required sensors

// sfg = pointer to top level (generally global) data structure for sensor fusion
int8_t FXAS21002_Init(struct PhysicalSensor *sensor, SensorFusionGlobals *sfg)
{
    uint8_t reg;
    int8_t status = SENSOR_ERROR_NONE;

    if (I2CReadByte(sensor->addr, FXAS21002_WHO_AM_I, &reg)) {
        sfg->Gyro.iWhoAmI = reg;
        switch (reg) {
        case FXAS21002_WHO_AM_I_WHOAMI_PROD_VALUE:
        case FXAS21002_WHO_AM_I_WHOAMI_PRE_VALUE:
        case FXAS21002_WHO_AM_I_WHOAMI_OLD_VALUE: 
            break;
        default:
            // whoAmI will retain default value of zero
            return SENSOR_ERROR_INIT; // return with error
        }
    } else {
        return SENSOR_ERROR_INIT; // return with error
    }

    // configure FXAS21000 or FXAS21002 depending on WHOAMI value read
    switch (sfg->Gyro.iWhoAmI) {
    case (FXAS21000_WHO_AM_I_VALUE):
        // Configure and start the FXAS21000 sensor.  This does multiple register writes
        // (see FXAS21009_Initialization definition above)
        status = Sensor_I2C_Write_List(&sensor->deviceInfo, sensor->addr, FXAS21000_INITIALIZATION );
        sfg->Gyro.iCountsPerDegPerSec = FXAS21000_COUNTSPERDEGPERSEC;
        sfg->Gyro.fDegPerSecPerCount = 1.0F / FXAS21000_COUNTSPERDEGPERSEC;
        break;
    case (FXAS21002_WHO_AM_I_WHOAMI_PRE_VALUE):
    case (FXAS21002_WHO_AM_I_WHOAMI_PROD_VALUE):
        status = Sensor_I2C_Write_List(&sensor->deviceInfo, sensor->addr, FXAS21002_INITIALIZATION );
        sfg->Gyro.iCountsPerDegPerSec = FXAS21002_COUNTSPERDEGPERSEC;
        sfg->Gyro.fDegPerSecPerCount = 1.0F / FXAS21002_COUNTSPERDEGPERSEC;
        break;
    }
    sfg->Gyro.iFIFOCount=0;
    sensor->isInitialized = F_USING_GYRO;
    sfg->Gyro.isEnabled = true;
    return (status);
}

// read FXAS21002 gyro over I2C
int8_t FXAS21002_Read(struct PhysicalSensor *sensor, SensorFusionGlobals *sfg)
{
    uint8_t     I2C_Buffer[6 * GYRO_FIFO_SIZE]; // I2C read buffer
    uint8_t      j;                              // scratch
    uint8_t     fifo_packet_count = 1;
    int32_t     status;
    int16_t     sample[3];

     if (sensor->isInitialized != F_USING_GYRO) {
      return SENSOR_ERROR_INIT;
    }

     // read the F_STATUS register (mapped to STATUS) and extract number of measurements available (lower 6 bits)
    status =  Sensor_I2C_Read(&sensor->deviceInfo, sensor->addr, FXAS21002_F_STATUS_READ, I2C_Buffer );
//    status = SENSOR_ERROR_NONE;
    if (status == SENSOR_ERROR_NONE) {
#ifdef SIMULATOR_MODE
        fifo_packet_count = 1;
#else
        fifo_packet_count = I2C_Buffer[0] & FXAS21002_F_STATUS_F_CNT_MASK ;
#endif
        // return if there are no measurements in the FIFO.
        // this will only occur when the calling frequency equals or exceeds GYRO_ODR_HZ
        if (fifo_packet_count == 0) return(SENSOR_ERROR_READ);
    } else {
      return (status);
    }
          // at this point there must be at least one measurement in the FIFO
          // available to read. handle the FXAS21000 and FXAS21002 differently
          // because only FXAS21002 supports WRAPTOONE feature.
          if (sfg->Gyro.iWhoAmI == FXAS21002_WHO_AM_I_WHOAMI_OLD_VALUE) {
//    if (true) {
            // read six sequential gyro output bytes
            FXAS21002_DATA_READ[0].readFrom = FXAS21002_OUT_X_MSB;
            FXAS21002_DATA_READ[0].numBytes = 6;

            // for FXAS21000, perform sequential 6 byte reads
            for (j = 0; j < fifo_packet_count; j++) {
              // read one set of measurements totalling 6 bytes
              status = Sensor_I2C_Read(&sensor->deviceInfo,
                                       sensor->addr, FXAS21002_DATA_READ,
                                       I2C_Buffer);

              if (status == SENSOR_ERROR_NONE) {
                // place the measurements read into the gyroscope buffer structure
                sample[CHX] = (I2C_Buffer[0] << 8) | I2C_Buffer[1];
                sample[CHY] = (I2C_Buffer[2] << 8) | I2C_Buffer[3];
                sample[CHZ] = (I2C_Buffer[4] << 8) | I2C_Buffer[5];
                conditionSample(sample);  // truncate negative values to -32767
                addToFifo((union FifoSensor*) &(sfg->Gyro), GYRO_FIFO_SIZE, sample);
            }
        }
    }   // end of FXAS21000 FIFO read
    else
    {  //Steady state when fusing at 40 Hz is 10 packets per cycle to read (gyro updates at 400 Hz). Takes 4 ms to read.
        // for FXAS21002, clear the FIFO in burst reads using WRAPTOONE feature, which decreases read time to 2 ms. 
        //Noticed that I2C reads > 126 bytes don't work, so limit the number of FIFO packets per burst read.
#define MAX_FIFO_PACKETS_PER_READ 11        
        FXAS21002_DATA_READ[0].readFrom = FXAS21002_OUT_X_MSB;
        while( (fifo_packet_count > 0)  && (status==SENSOR_ERROR_NONE)) {
            if( MAX_FIFO_PACKETS_PER_READ < fifo_packet_count ) {
               FXAS21002_DATA_READ[0].numBytes = MAX_FIFO_PACKETS_PER_READ * 6;
               fifo_packet_count -= MAX_FIFO_PACKETS_PER_READ;
            }else {
                FXAS21002_DATA_READ[0].numBytes = fifo_packet_count * 6;
                fifo_packet_count = 0;
            }
            status = Sensor_I2C_Read(&sensor->deviceInfo,
                                     sensor->addr, FXAS21002_DATA_READ,
                                     I2C_Buffer);
            if (status==SENSOR_ERROR_NONE) {
                for (j = 0; j < FXAS21002_DATA_READ[0].numBytes; j+=6) {
                    // place the measurements read into the gyroscope buffer structure
                    sample[CHX] = (I2C_Buffer[j + 0] << 8) | I2C_Buffer[j + 1];
                    sample[CHY] = (I2C_Buffer[j + 2] << 8) | I2C_Buffer[j + 3];
                    sample[CHZ] = (I2C_Buffer[j + 4] << 8) | I2C_Buffer[j + 5];
                    conditionSample(sample);  // truncate negative values to -32767
                    addToFifo((union FifoSensor*) &(sfg->Gyro), GYRO_FIFO_SIZE, sample);
                }
            }
        }
    }   // end of optimized FXAS21002 FIFO read

    return status;
        }

// Each entry in a RegisterWriteList is composed of: register address, value to write, bit-mask to apply to write (0 enables)
const registerwritelist_t   FXAS21002_IDLE[] =
{
  // Reset to Standby
  { FXAS21000_CTRL_REG1, 0x00, 0x00 },
    __END_WRITE_DATA__
};

// FXAS21002_Idle places the gyro into READY mode (wakeup time = 1/ODR+5ms)
int8_t FXAS21002_Idle(struct PhysicalSensor *sensor, SensorFusionGlobals *sfg)
{
    int32_t     status;
    if(sensor->isInitialized == F_USING_GYRO) {
        status = Sensor_I2C_Write_List(&sensor->deviceInfo, sensor->addr, FXAS21002_IDLE );
        sensor->isInitialized = 0;
        sfg->Gyro.isEnabled = false;
    } else {
      return SENSOR_ERROR_INIT;
    }
    return status;
}
#endif

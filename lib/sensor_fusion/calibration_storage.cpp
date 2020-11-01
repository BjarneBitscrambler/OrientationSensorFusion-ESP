
/*! \file calibration_storage.c
    \brief Provides functions to store calibration to NVM

    Written for use on Arduino-Espressif environment where EEPROM library available.
	Bjarne Hansen  2020-10-26
*/
#include <stdio.h>
#include <EEPROM.h>
#include "debug_print.h"
#include "sensor_fusion.h"
#include "calibration_storage.h"

#define CALIBRATION_STORAGE_SIZE_BYTES 256
#define CALIBRATION_BUF_MAGNETIC_START 0
#define CALIBRATION_BUF_MAGNETIC_HDR_SIZE 4
#define CALIBRATION_BUF_MAGNETIC_HDR_MAGIC 0x12345678
#define CALIBRATION_BUF_MAGNETIC_VAL_SIZE 64
#define CALIBRATION_BUF_MAGNETIC_TOT_SIZE (CALIBRATION_BUF_MAGNETIC_HDR_SIZE + CALIBRATION_BUF_MAGNETIC_VAL_SIZE)
#define CALIBRATION_BUF_GYRO_START (CALIBRATION_BUF_MAGNETIC_TOT_SIZE)
#define CALIBRATION_BUF_GYRO_HDR_SIZE 4
#define CALIBRATION_BUF_GYRO_HDR_MAGIC 0x12345678
#define CALIBRATION_BUF_GYRO_VAL_SIZE 12
#define CALIBRATION_BUF_GYRO_TOT_SIZE (CALIBRATION_BUF_GYRO_HDR_SIZE + CALIBRATION_BUF_GYRO_VAL_SIZE)
#define CALIBRATION_BUF_ACCEL_START (CALIBRATION_BUF_MAGNETIC_TOT_SIZE + CALIBRATION_BUF_GYRO_TOT_SIZE)
#define CALIBRATION_BUF_ACCEL_HDR_SIZE 4
#define CALIBRATION_BUF_ACCEL_HDR_MAGIC 0x12345678
#define CALIBRATION_BUF_ACCEL_VAL_SIZE 84
#define CALIBRATION_BUF_ACCEL_TOT_SIZE (CALIBRATION_BUF_ACCEL_HDR_SIZE + CALIBRATION_BUF_ACCEL_VAL_SIZE)
#define CALIBRATION_NO_MAGIC 0xdeadbeef
#if CALIBRATION_STORAGE_SIZE_BYTES < (CALIBRATION_BUF_MAGNETIC_TOT_SIZE + CALIBRATION_BUF_GYRO_TOT_SIZE + CALIBRATION_BUF_ACCEL_TOT_SIZE)
	#error insufficient space allocated for calibration buffer
#endif

//fetch the Magnetic calibration values from non-volatile memory.
//If cal values are unavailable, returns false. If successful, returns true.
bool GetMagCalibrationFromNVM( float *cal_values ) {
    if( NULL == cal_values ) {
      return false;
    }
#if F_USING_MAG
    //header of each calibration block type, containing 
    //magic value if valid calibration has been previously stored.

    EEPROM.begin(
        CALIBRATION_STORAGE_SIZE_BYTES);  // access chunk of emulated EEPROM
    
    //check for the expected header - if not found, then calibration invalid
    //this code is a bit hacky: can't use an arbitrary # of bytes in header.
    uint8_t buf_magic[CALIBRATION_BUF_MAGNETIC_HDR_SIZE];	
    EEPROM.readBytes(CALIBRATION_BUF_MAGNETIC_START, buf_magic,
                     CALIBRATION_BUF_MAGNETIC_HDR_SIZE);
    uint32_t magic_value = CALIBRATION_BUF_MAGNETIC_HDR_MAGIC;
    uint8_t *pSrc = (uint8_t *)&magic_value;
    for (int i = 0; i < CALIBRATION_BUF_MAGNETIC_HDR_SIZE; i++) {
      if( *pSrc != buf_magic[i] ) {
        return false;
      }
      ++pSrc;
    }

    //read the valid calibration into provided destination
    EEPROM.readBytes(
        CALIBRATION_BUF_MAGNETIC_START + CALIBRATION_BUF_MAGNETIC_HDR_SIZE,
        cal_values, CALIBRATION_BUF_MAGNETIC_VAL_SIZE);

    EEPROM.end();
    return true;
#endif  // if F_USING_MAG
    return false;
}//end GetMagCalibrationFromNVM()

bool GetGyroCalibrationFromNVM( float *cal_values ) {
    if( NULL == cal_values ) {
      return false;
    }
#if F_USING_GYRO
    //header of each calibration block type, containing 
    //magic value if valid calibration has been previously stored.
    EEPROM.begin(
        CALIBRATION_STORAGE_SIZE_BYTES);  // access chunk of emulated EEPROM
    
    //check for the expected header - if not found, then calibration invalid
    //this code is a bit hacky: can't use an arbitrary # of bytes in header.
    uint8_t buf_magic[CALIBRATION_BUF_GYRO_HDR_SIZE];	
    EEPROM.readBytes(CALIBRATION_BUF_GYRO_START, buf_magic,
                     CALIBRATION_BUF_GYRO_HDR_SIZE);
    uint32_t magic_value = CALIBRATION_BUF_GYRO_HDR_MAGIC;
    uint8_t *pSrc = (uint8_t *)&magic_value;
    for (int i = 0; i < CALIBRATION_BUF_GYRO_HDR_SIZE; i++) {
      if( *pSrc != buf_magic[i] ) {
        return false;
      }
      ++pSrc;
    }

    //read the valid calibration into provided destination
    EEPROM.readBytes(
        CALIBRATION_BUF_GYRO_START + CALIBRATION_BUF_GYRO_HDR_SIZE,
        cal_values, CALIBRATION_BUF_GYRO_VAL_SIZE);

    EEPROM.end();
    return true;
#endif  // if F_USING_GYRO
    return false;
}//end GetGyroCalibrationFromNVM()

bool GetAccelCalibrationFromNVM( float *cal_values ) {
    if( NULL == cal_values ) {
      return false;
    }
#if F_USING_ACCEL
    //header of each calibration block type, containing 
    //magic value if valid calibration has been previously stored.
    EEPROM.begin(
        CALIBRATION_STORAGE_SIZE_BYTES);  // access chunk of emulated EEPROM
    
    //check for the expected header - if not found, then calibration invalid
    //this code is a bit hacky: can't use an arbitrary # of bytes in header.
    uint8_t buf_magic[CALIBRATION_BUF_ACCEL_HDR_SIZE];	
    EEPROM.readBytes(CALIBRATION_BUF_ACCEL_START, buf_magic,
                     CALIBRATION_BUF_ACCEL_HDR_SIZE);
    uint32_t magic_value = CALIBRATION_BUF_ACCEL_HDR_MAGIC;
    uint8_t *pSrc = (uint8_t *)&magic_value;
    for (int i = 0; i < CALIBRATION_BUF_ACCEL_HDR_SIZE; i++) {
      if( *pSrc != buf_magic[i] ) {
        return false;
      }
      ++pSrc;
    }

    //read the valid calibration into provided destination
    EEPROM.readBytes(
        CALIBRATION_BUF_ACCEL_START + CALIBRATION_BUF_ACCEL_HDR_SIZE,
        cal_values, CALIBRATION_BUF_ACCEL_VAL_SIZE);

    EEPROM.end();
    return true;
#endif  // if F_USING_ACCEL
    return false;
}//end GetAccelCalibrationFromNVM()


void SaveMagCalibrationToNVM(SensorFusionGlobals *sfg)
{
#if F_USING_MAG
	uint8_t *pSrc, *pDst;		// scratch pointers
    uint8_t *buf_NVM = NULL;	//pointer into the non-volatile memory buffer

    EEPROM.begin(
        CALIBRATION_STORAGE_SIZE_BYTES);  // access chunk of emulated EEPROM
    buf_NVM = EEPROM.getDataPtr();

    //modify magnetic, leave existing gyro and accelerometer calibrations in buffer
    pDst = buf_NVM + CALIBRATION_BUF_MAGNETIC_START;

    // write buffer with magnetic header + calibration in bytes 0 to 67
	//bytes[0-3]: four byte header denoting magnetic calibration present
    uint32_t itmp32 = CALIBRATION_BUF_MAGNETIC_HDR_MAGIC;
    pSrc = (uint8_t *)&itmp32;
    for (int i = 0; i < CALIBRATION_BUF_MAGNETIC_HDR_SIZE; i++) {
		*pDst = *pSrc;
        ++pDst;
        ++pSrc;
    }
    //bytes[4-67]: magnetic calibration: 15x float + 1x int32 subtotal 64 bytes
    pSrc = (uint8_t *)&(sfg->MagCal);
    for (int i = 0; i < CALIBRATION_BUF_MAGNETIC_VAL_SIZE; i++) {
		*pDst = *pSrc;
        ++pDst;
        ++pSrc;
	}
    // write the whole buffer contents to NVM
    if( ! EEPROM.commit() ) {
		debug_log("EEPROM write mag cal failed\n");
	}
    EEPROM.end();
#endif  // if F_USING_MAG
        return;
}

void SaveGyroCalibrationToNVM(SensorFusionGlobals *sfg)
{
#if F_USING_GYRO && (F_9DOF_GBY_KALMAN || F_6DOF_GY_KALMAN)
	uint8_t *pSrc, *pDst;		// scratch pointers
    uint8_t *buf_NVM = NULL;	//pointer into the non-volatile memory buffer

    EEPROM.begin(
        CALIBRATION_STORAGE_SIZE_BYTES);  // access chunk of emulated EEPROM
    buf_NVM = EEPROM.getDataPtr();

    //modify gyro, leave existing magnetic and accelerometer calibrations in buffer
    pDst = buf_NVM + CALIBRATION_BUF_GYRO_START;

    // write buffer with gyro header + calibration in bytes 0 to 15
	//bytes[0-3]: four byte header denoting gyro calibration present
    uint32_t itmp32 = CALIBRATION_BUF_GYRO_HDR_MAGIC;
    pSrc = (uint8_t *)&itmp32;
    for (int i = 0; i < CALIBRATION_BUF_GYRO_HDR_SIZE; i++) {
		*pDst = *pSrc;
        ++pDst;
        ++pSrc;
    }
	//bytes [4-15]: 3 gyro offset floats sub totalling 12 bytes
#if F_9DOF_GBY_KALMAN
	pSrc = (uint8_t *) sfg->SV_9DOF_GBY_KALMAN.fbPl;
#elif F_6DOF_GY_KALMAN
	pSrc = (uint8 *) sfg->SV_6DOF_GY_KALMAN.fbPl;
#endif
    pSrc = (uint8_t *)&(sfg->MagCal);
    for (int i = 0; i < CALIBRATION_BUF_GYRO_VAL_SIZE; i++) {
		*pDst = *pSrc;
        ++pDst;
        ++pSrc;
	}
    // write the whole buffer contents to NVM
    if( ! EEPROM.commit() ) {
		debug_log("EEPROM write gyro cal failed\n");
	}
    EEPROM.end();
#endif
	return;
}

void SaveAccelCalibrationToNVM(SensorFusionGlobals *sfg)
{
#if F_USING_ACCEL
	uint8_t *pSrc, *pDst;		// scratch pointers
    uint8_t *buf_NVM = NULL;	//pointer into the non-volatile memory buffer

    EEPROM.begin(
        CALIBRATION_STORAGE_SIZE_BYTES);  // access chunk of emulated EEPROM
    buf_NVM = EEPROM.getDataPtr();

    //modify accel, leave existing magnetic and gyro calibrations in buffer
    pDst = buf_NVM + CALIBRATION_BUF_ACCEL_START;

    // write buffer with accel header + calibration in bytes 0 to 87
	//bytes[0-3]: four byte header denoting gyro calibration present
    uint32_t itmp32 = CALIBRATION_BUF_ACCEL_HDR_MAGIC;
    pSrc = (uint8_t *)&itmp32;
    for (int i = 0; i < CALIBRATION_BUF_ACCEL_HDR_SIZE; i++) {
		*pDst = *pSrc;
        ++pDst;
        ++pSrc;
    }
	//bytes [4-87]: 21 precision accelerometer calibration floats subtotalling 84 bytes
	pSrc = (uint8_t *) &(sfg->AccelCal);
    for (int i = 0; i < CALIBRATION_BUF_ACCEL_VAL_SIZE; i++) {
		*pDst = *pSrc;
        ++pDst;
        ++pSrc;
	}
    // write the whole buffer contents to NVM
    if( ! EEPROM.commit() ) {
		debug_log("EEPROM write accel cal failed\n");
	}
    EEPROM.end();
#endif
	return;
}

void EraseMagCalibrationFromNVM(void)
{
	uint8_t *pSrc, *pDst;		// scratch pointers
    uint8_t *buf_NVM = NULL;	//pointer into the non-volatile memory buffer

    EEPROM.begin(
        CALIBRATION_STORAGE_SIZE_BYTES);  // access chunk of emulated EEPROM
    buf_NVM = EEPROM.getDataPtr();

    pDst = buf_NVM + CALIBRATION_BUF_MAGNETIC_START;
	//bytes[0-3]: four byte header denoting magnetic calibration present
    uint32_t itmp32 = CALIBRATION_NO_MAGIC;
    pSrc = (uint8_t *)&itmp32;
    for (int i = 0; i < CALIBRATION_BUF_MAGNETIC_HDR_SIZE; i++) {
		*pDst = *pSrc;
        ++pDst;
        ++pSrc;
    }
    // write the whole buffer contents to NVM
    if( ! EEPROM.commit() ) {
		debug_log("EEPROM clear magnetic cal failed\n");
	}
    EEPROM.end();

	return;
}

void EraseGyroCalibrationFromNVM(void)
{
	uint8_t *pSrc, *pDst;		// scratch pointers
    uint8_t *buf_NVM = NULL;	//pointer into the non-volatile memory buffer

    EEPROM.begin(
        CALIBRATION_STORAGE_SIZE_BYTES);  // access chunk of emulated EEPROM
    buf_NVM = EEPROM.getDataPtr();

    pDst = buf_NVM + CALIBRATION_BUF_GYRO_START;
	//bytes[0-3]: four byte header denoting magnetic calibration present
    uint32_t itmp32 = CALIBRATION_NO_MAGIC;
    pSrc = (uint8_t *)&itmp32;
    for (int i = 0; i < CALIBRATION_BUF_GYRO_HDR_SIZE; i++) {
		*pDst = *pSrc;
        ++pDst;
        ++pSrc;
    }
    // write the whole buffer contents to NVM
    if( ! EEPROM.commit() ) {
		debug_log("EEPROM clear gyro cal failed\n");
	}
    EEPROM.end();

	return;
}

void EraseAccelCalibrationFromNVM(void)
{
	uint8_t *pSrc, *pDst;		// scratch pointers
    uint8_t *buf_NVM = NULL;	//pointer into the non-volatile memory buffer

    EEPROM.begin(
        CALIBRATION_STORAGE_SIZE_BYTES);  // access chunk of emulated EEPROM
    buf_NVM = EEPROM.getDataPtr();

    pDst = buf_NVM + CALIBRATION_BUF_ACCEL_START;
	//bytes[0-3]: four byte header denoting magnetic calibration present
    uint32_t itmp32 = CALIBRATION_NO_MAGIC;
    pSrc = (uint8_t *)&itmp32;
    for (int i = 0; i < CALIBRATION_BUF_ACCEL_HDR_SIZE; i++) {
		*pDst = *pSrc;
        ++pDst;
        ++pSrc;
    }
    // write the whole buffer contents to NVM
    if( ! EEPROM.commit() ) {
		debug_log("EEPROM clear magnetic cal failed\n");
	}
    EEPROM.end();

	return;
}

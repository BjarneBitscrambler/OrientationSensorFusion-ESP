#ifndef CALIBRATION_STORAGE_H
#define CALIBRATION_STORAGE_H

#ifdef __cplusplus
extern "C" {
#endif
  
/*! \file calibration_storage.h
    \brief Provides functions to store calibration to NVM
*/
bool GetMagCalibrationFromNVM( float *cal_values );
bool GetGyroCalibrationFromNVM( float *cal_values );
bool GetAccelCalibrationFromNVM( float *cal_values );
void SaveMagCalibrationToNVM(SensorFusionGlobals *sfg);
void SaveGyroCalibrationToNVM(SensorFusionGlobals *sfg);
void SaveAccelCalibrationToNVM(SensorFusionGlobals *sfg);
void EraseMagCalibrationFromNVM(void);
void EraseGyroCalibrationFromNVM(void);
void EraseAccelCalibrationFromNVM(void);

#ifdef __cplusplus
}
#endif

#endif //CALIBRATION_STORAGE_H

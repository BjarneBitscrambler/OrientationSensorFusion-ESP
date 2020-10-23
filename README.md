# OrientationSensorTesting
Contains test plans, results, to-do lists relating to testing of an orientation sensor.

## Sensor
NXP 9DOF sensor combination (magnetometer, accelerometer, gyroscope) consisting of FXOS8700 + FXAS21002 (e.g. Adafruit #3463 breakout board).

## Background
The sensor is used with the Signal K / SensESP project (https://github.com/SignalK/SensESP) to provide orientation data (e.g. magnetic heading, roll, and pitch) on a vessel.
The aim of the testing is to determine characteristics of the sensor such as accuracy, repeatability, and response time.

## Apparatus
A bicycle wheel (composed of an aluminum rim and stainless-steel spokes, and steel axle) is mounted on a wooden plank. 
The sensor and associated circuit board is lashed to the spokes near the rim, with the sensor about 24cm away from the ferromagnetic materials in the axle.
A counterweight is attached to the wheel opposite the circuit board so the wheel can rotate freely and without excessive imbalance.

## Testing
### Stationary Response
### Step Response

## Known Issues
1. Intermittently the output from the filter function will drift steadily and continuously away from the known heading, even when the sensor is held stationary.
Example as observed by tedenda: 
![alt-text](https://user-images.githubusercontent.com/4216986/96929437-68397280-14ba-11eb-851c-1f752c096e8b.png "as observed by tedenda")

Limited testing suggests that it is not caused by lack of calibration.

## Resources
1. https://github.com/SignalK/signalk-imu
2. https://www.tindie.com/products/onehorse/ultimate-sensor-fusion-solution-mpu9250/
3. kriswiner's Github repositories, including: https://github.com/kriswiner/EM7180_SENtral_sensor_hub/wiki/K.-Limits-of-Absolute-Heading-Accuracy-Using-Inexpensive-MEMS-Sensors
 and https://github.com/kriswiner/MPU6050/wiki/Affordable-9-DoF-Sensor-Fusion
4. [Estimation of heading using magnetometer and GPS](https://www.diva-portal.org/smash/get/diva2:648760/FULLTEXT01.pdf)
5. [Performance Improvement of Inertial Navigation System byUsing Magnetometer with Vehicle Dynamic Constraints](http://downloads.hindawi.com/journals/js/2015/435062.pdf)




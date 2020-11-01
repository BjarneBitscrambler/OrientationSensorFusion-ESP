# Orientation Sensor Fusion on ESP platforms
Contains source code for performing orientation sensor fusion. The code is based on NXP's version 7.20 algorithm (released under BSD 3-Clause license), ported to run on the ESP32 processor (and possibly other Espressif processors). The present software uses the NXP FXOS8700 and FXAS21002C sensor ICs.

Information about test plans, results, to-do lists for the orientation sensors and software can be found on this repository's Wiki.

## Sensor
The present software works with the NXP 9DOF sensor combination (magnetometer, accelerometer, gyroscope) consisting of FXOS8700 + FXAS21002 (e.g. Adafruit #3463 breakout board). Only the I2C interface has been implemented and tested; a SPI interface is possible with additional work and testing.

## Background
The sensor is used with the Signal K / SensESP project (https://github.com/SignalK/SensESP) to provide orientation data (e.g. magnetic heading, roll, and pitch) on a vessel.

## Where To Find...
- test plans, data, results, etc are on [this project's Wiki](https://github.com/BjarneBitscrambler/OrientationSensorFusion-ESP/wiki)
- code for running NXP's version 7 sensor fusion is under the Code tab of this repository. It hasn't been integrated to work with the SensESP project yet.
- code for orientation sensor fusion using Adafruit's AHRS port of NXP's version 4.2 algorithm is on the [Upstream](https://github.com/SignalK/SensESP) and [author's](https://github.com/BjarneBitscrambler/SensESP) project pages

## Contributions
Use the Issues and Pull Request tabs on this repository if you have suggestions or wish to contribute to this project.

## How-To
To use this library follow these steps (untested - let me know of any changes you needed to make to get things to work on your setup):
- create a local clone of this repository on your computer
- setup the PlatformIO development environment
- create a new PlatformIO project, selecting the *Board:* `Espressif ESP-WROVER-KIT` and *Framework:* `Arduino` (other ESP32 boards should work without changes; other Espressif CPUs like the ESP8266 may need some code adjustments - not tested)
- copy `main.cpp` into your new PlatformIO project's `/src` folder
- copy the `/lib/sensor_fusion` files into your new project's `/lib/sensor_fusion` folder
- edit your new project's `platformio.ini` file for your specific board and environment. Use this project's `platformio.ini` file as an example.
- edit `/lib/sensor_fusion/board.h` to reflect your particular hardware. The I2C pins connecting your processor to your sensor ICs will likely be different, and you may also need to change the I2C addresses that the ICs are configured for.
- compile and download to your processor

The out-of-the-box software is configured to send data packets containing the sensor fusion results at a rate of 40 Hz over the processor's Serial UART interface (connected to the USB port on my WROVER development kit). These packets are formatted for NXP's **Sensor Fusion Toolbox** Windows application (available for download from NXP at no cost) which will display the data and can even be used to send commands back to the processor running the fusion algorithms. See the User's Guide for the Toolbox for details.


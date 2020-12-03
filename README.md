# Sensor Fusion Library

## Introduction
An easy-to-use interface to the **NXP Sensor Fusion version 7** algorithms.

It is configured to work with the Adafruit breakout board #3643
using the *NXP FXOS8700* magnetometer/accelerometer and *FXAS21002* gyroscope
sensor ICs, but can be modified to work with other sensors having an I2C
interface. With additional modification, it can also work with SPI interface
sensors.

The library runs on Espressif's ESP32 and ESP8266 processors and outputs
orientation data using the serial and WiFi interfaces.

A C++ class provides simple access to the most common sensor fusion
functions, but it is also possible to directly interface with the library
methods contained in the underlying C files, which are based on those provided
by NXP in their version 7.20 release.


## Background
Orientation sensing using combined accelerometer + gyroscope + magnetometer sensors has become quite accurate when coupled with the right sensor fusion software. [NXP](https://www.nxp.com/) manufactures these types of sensors, and have written an excellent sensor fusion library for their Kinetis 32-bit microcontrollers. Their library, which is released under the BSD-3-Clause license, was ported in 2015 to the Arduino environment by Adafruit as their [AHRS (Attitude and Heading Reference System)](https://learn.adafruit.com/how-to-fuse-motion-sensor-data-into-ahrs-orientation-euler-quaternions). The AHRS port uses NXP's version 4.2 fusion code.

A newer version of the NXP sensor fusion library (version 7.2) is available, which is what this present project is using. This newer library has several improvements, including the ability to perform magnetic calibration while in use (as opposed to needing a separate software tool).

My motivation for porting NXP's library is to create an orientation sensor for marine use, using off-the-shelf hardware and the [Signal K / SensESP project](https://github.com/SignalK/SensESP) to provide orientation data (e.g. magnetic heading, roll, and pitch) on a vessel. I have targeted this project for the Espressif ESP32 microcontroller and the NXP sensors. 


## Sensors
The present software works with the NXP 9DOF (9 Degrees-Of-Freedom) sensor combination consisting of **FXOS8700 magnetometer + accelerometer** and **FXAS21002 gyroscope**. These are conveniently available mounted together on the **Adafruit #3463 breakout** board. 

Other sensors could be used. Note that only the I2C interface has been implemented and tested; a SPI interface is possible with additional work and testing.

## Processor
The present software is written for the ESP32 and ESP8266 processors. With some rewriting of the I2C and timing routines, the code can be ported to other processors.

## Dependencies
The fusion code and associated project files have been written for and
tested in the *PlatformIO* development environment, as an *Arduino* framework
project for an ESP32 board.

In addition to the standard Arduino environment, this project uses these libraries:
- **Wire (I2C)** library
- **WiFi** libraries (*only needed if WiFi output is enabled*)

## Where To Find...
- **Test plans, data, results**, etc are on [this project's Github Wiki](https://github.com/BjarneBitscrambler/OrientationSensorFusion-ESP/wiki)
- **NXP's version 7 sensor fusion** for ESP32 processors is under the [Code tab](https://github.com/BjarneBitscrambler/OrientationSensorFusion-ESP) of this Github repository. It is fully functional with [NXP's Windows-based Sensor Fusion Toolbox](https://www.nxp.com/webapp/sps/download/license.jsp?colCode=SENSORFUSIONREV7) software application. 
- **Adafruit's AHRS port of NXP's version 4.2 sensor fusion** as integrated with the SensESP project is on the [Upstream](https://github.com/SignalK/SensESP) and [author's](https://github.com/BjarneBitscrambler/SensESP) project pages

## Contributions
Use the *Issues* and *Pull Request* tabs on this project's Github repository if you have suggestions or wish to contribute to this project.

## How-To Use
To use this library follow these steps (some untested - let me know of any changes you needed to make to get things to work on your setup):
- setup the PlatformIO development environment
- create a new PlatformIO project, selecting the *Board:* `Espressif ESP-WROVER-KIT` and *Framework:* `Arduino` (other ESP32 boards should work without changes; Espressif CPUs like the ESP8266 may need some code adjustments - not tested)

Now follow either of these two methods to bring in the library files:
### Method 1
- create a local clone of this repository on your computer
- from your local repository, copy `/examples/fusion_text_output.cc` into your new PlatformIO project's `/src` folder. You may want to rename it `main.cc` to remind yourself that it contains the `setup()` and `loop()` functions.
- from your local repository, copy the files and subfolders of `/src/*` into your new project's `/lib/sensor_fusion` folder

### Method 2
- copy this project's `/examples/fusion_text_output.cc` into your new PlatformIO project's `/src` folder
- in your new project's `platformio.ini` file, add the SensorFusionLibrary **TODO fixup**

Then:  
- edit your new project's `platformio.ini` file for your specific board and environment. Use this project's `platformio.ini` file as an example.
- edit `/lib/sensor_fusion/board.h` to reflect your particular hardware. The I2C pins connecting your processor to your sensor ICs will likely be different, and you may also need to change the I2C addresses that the ICs are configured for.
- compile and download to your processor

The file`/lib/sensor_fusion/build.h` contains defines for various functionality, such as whether the software outputs its data via hardware serial UART or WiFi TCP connections, or both (default). Edit this file as desired, but note that not all combinations of features may be valid or been tested.

### Initial Text Output
To confirm that the software is communicating with your sensors, observe the serial port output of the ESP processor. Use a USB connection to a PC running a terminal program at 115200 baud, 8 bits, No parity, 1 stop bit. Several progress messages should appear as the fusion software configures the sensors. Once the algorithm is running, lines of text containing a timestamp and orientation data should scroll by. See the `main.cc` program's `loop()` for details.

However, the best way to visualize operation of the orientation algorithm is by using the NXP Sensor Fusion Toolbox.

### NXP Sensor Fusion Toolbox
The out-of-the-box software is configured to send data packets containing the sensor fusion results at a rate of 40 Hz over the processor's Serial UART interface (connected to the USB port on my WROVER development kit). These packets are formatted for NXP's **Sensor Fusion Toolbox** Windows application (available for download from NXP at no cost) which will display the data and can even be used to send commands back to the processor running the fusion algorithms. See the User's Guide under the Help tab of the Toolbox for details.

The **Toolbox** when working shows a graphic of a PCB rotating on the screen in synchronization with motion of your own board. If there is no motion at all, then check that the data packets are arriving on the expected COM: port of your computer. A terminal program (like HyperTerminal or PuTTY) can help display traffic on a COM: port, but note that the data packets are in binary format (not ASCII text) so you won't be able to interpret them visually. If the **Toolbox** shows motion but it is jerky or reversed from the actual board motion, then likely one or more of your board's axes are not oriented according to how the fusion software expects. Different sensor board manufacturers will have placed the sensor ICs in orientations particular to their own needs.  The file `hal_axis_remap.c` is used to invert or swap axes to conform to what the fusion algorithm expects. For more details, see that file, and also NXP's *Application Note AN5017 (Coordinate Systems)*.

### WiFi Data Streaming
Because testing an orientation sensor with a USB cable tethering it to your development computer is a pain, the software also supports streaming the data over WiFi. In the main `setup()` code, a WiFi AP (Access Point) is started, which means the ESP processor will broadcast its SSID and you should be able to connect to it with your development system using the password you provide in the `main.cc` file. Once a WiFi connection is established, you can open a TCP connection to port 23 of the ESP and the orientation data will then stream over your TCP connection.  A few hints:
- view the ESP's serial output (e.g. using that USB connection) to find out what IP address the ESP has assigned itself
- on a linux system, an easy way to make a TCP connection is the command `telnet 192.168.4.1`, where you replace the example IP address with the one the ESP has indicated in its serial output. 
- Once you have noted the IP address, it shouldn't change between reboots of the ESP.
- piping the data from the TCP connection to a serial port on a Windows computer (to make it available for the *Sensor Toolbox*) isn't trivial unfortunately. I tried two applications: **H.W. Virtual Serial Port** and **TCP-COM**.  Both are free for time-limited trial copies. The first one exhibited intermittent data dropouts, which confused the *Sensor Toolbox* quite badly. TCP-COM was better-behaved and has worked well for several hours. There may be better alternatives - if you know of one, let me know and I'll list it here.

I did earlier try having the ESP connect as a client to our WiFi router, rather than acting as an AP itself. Unfortunately, this caused delays in delivery of the streamed traffic that would intermittently freeze the *Sensor Toolbox*. Performing a `ping` from the development computer to the ESP showed trip times sometimes exceeded 1000 ms when going through the router.  So, using the ESP as an AP is better for timely data delivery on my hardware.

Using WiFi (even when ESP is acting as AP) *does* introduce noticeable lag in the Toolbox graphic response, compared to a USB connection. It looks like about a 200 ms lag on my system.

### Additional Debugging
You can use the GPIO output that toggles each time through the data collection and sending loop to confirm whether your ESP is collecting and transmitting data regularly. Using the default software, the output should toggle every 25 ms (i.e. a 20 Hz square wave). See `fusion_text_output.cc` for details.

## Author
Bjarne Hansen

## License
Copyright (c) 2020, Bjarne Hansen
All rights reserved.

SPDX-License-Identifier: BSD-3-Clause




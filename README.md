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
To use this library follow these steps (some untested - let me know of any changes you needed to make to get things to work on your setup):
- create a local clone of this repository on your computer
- setup the PlatformIO development environment
- create a new PlatformIO project, selecting the *Board:* `Espressif ESP-WROVER-KIT` and *Framework:* `Arduino` (other ESP32 boards should work without changes; other Espressif CPUs like the ESP8266 may need some code adjustments - not tested)
- copy `main.cpp` into your new PlatformIO project's `/src` folder
- copy the `/lib/sensor_fusion` files into your new project's `/lib/sensor_fusion` folder
- edit your new project's `platformio.ini` file for your specific board and environment. Use this project's `platformio.ini` file as an example.
- edit `/lib/sensor_fusion/board.h` to reflect your particular hardware. The I2C pins connecting your processor to your sensor ICs will likely be different, and you may also need to change the I2C addresses that the ICs are configured for.
- compile and download to your processor

### NXP Sensor Toolbox
The out-of-the-box software is configured to send data packets containing the sensor fusion results at a rate of 40 Hz over the processor's Serial UART interface (connected to the USB port on my WROVER development kit). These packets are formatted for NXP's **Sensor Fusion Toolbox** Windows application (available for download from NXP at no cost) which will display the data and can even be used to send commands back to the processor running the fusion algorithms. See the User's Guide under the Help tab of the Toolbox for details.

### WiFi Data Streaming
Because testing an orientation sensor with a USB cable tethering it to your development computer is a pain, the software now also supports streaming the data over WiFi. In the `main.cpp setup()` code, a WiFi AP (Access Point) is started, which means the ESP processor will broadcast it's SSID and you should be able to connect to it with your development system, using the password you provide in `main.cpp`. Once a WiFi connection is established, you can open a TCP connection to port 23 of the ESP and the orientation data will then stream over your TCP connection.  A few hints:
- view the ESP's serial output (e.g. using that USB connection) to find out what IP address the ESP has assigned itself
- on a linux system, an easy way to make and test the connection is the command `telnet 192.168.4.1`, where you replace the example IP address with the one the ESP has indicated in it's serial output. 
- Once you have noted the IP address, it shouldn't change between reboots of the ESP.
- piping the data from the TCP connection to a serial port on a Windows computer (to make it available for the Sensor Toolbox) isn't trivial, unfortunately. I tried two applications: **H.W. Virtual Serial Port** and **TCP-COM**.  Both are free for time-limited trial copies. The first one exhibited intermittent data dropouts, which confused the Sensor Toolbox quite badly. TCP-COM was better-behaved and has worked well for several hours. There may be better alternatives - if you know of one, let me know and I'll list it here.

I did earlier try having the ESP connect as a client to our WiFi router, rather than acting as an AP itself. Unfortunately, this caused delays in delivery of the streamed traffic that would intermittently freeze the Sensor Toolbox (another symptom was performing a `ping` from the development computer to the ESP - trip times sometimes exceeded 1000 ms when going through the router).  So, using the ESP as an AP was better for the timeliness of data delivery, on my hardware.

### Additional Debugging
You can use the GPIO output that toggles each time through the data collection and sending loop to confirm whether your ESP is collecting and transmitting data regularly. Using the default software, the output should toggle every 25 ms (i.e. a 20 Hz square wave). See `main.cpp` for details.


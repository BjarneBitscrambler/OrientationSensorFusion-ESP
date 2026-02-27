/**
 * @file main.cc
 * @brief Example main program, demonstrating how to use SensorFusion class.
 * 
 * This file creates a SensorFusion object, and runs a loop that produces
 * orientation data every 25ms. Output is possible in NXP SensorToolbox
 * application format, and also in user-defined format, depending on which
 * functions are called. Both Serial UART and WiFi output are supported. Because
 * this file is also being used for development of the fusion library, various
 * features may be commented in/out in the latest version found on Github. You
 * may want to keep a local copy that doesn't get modified each time you refresh
 * your local repository.
 */

#include <Arduino.h>
#include <sstream>
#include <string>
#include <esp_log.h>

#ifdef ESP8266
  #include <ESP8266WiFi.h>
  #include <ESP8266WiFiAP.h>
#endif
#ifdef ESP32
  #include <WiFi.h>
  #include <WiFiAP.h>
#endif
#include <WiFiClient.h>

// Sensor Fusion Headers
#include "sensor_fusion_class.h"
#include "board.h"   // hardware-specific settings. Edit as needed for board & sensors.
#include "build.h"   // sensor fusion configuration options. Edit as needed.

// UART details for data streaming and debug messages. */
#ifndef BOARD_DEBUG_UART_BAUDRATE
#define BOARD_DEBUG_UART_BAUDRATE 115200
#endif

// I2C details - indicate which pins the sensor is connected to
//Pin numbers are specified using the GPIO## scheme, not the Arduino D## scheme.
#ifdef ESP8266
  #define PIN_I2C_SDA   (12)  //Adjust to your board. A value of -1
  #define PIN_I2C_SCL   (14)  // will use default Arduino pins.
#endif
#ifdef ESP32
  #define PIN_I2C_SDA   (23)  //Adjust to your board. A value of -1 (11) for Nano (23) for ESP32-WROVER
  #define PIN_I2C_SCL   (25)  // will use default Arduino pins.     (12) for nano (25) for ESP32-WROVER
#endif
// sensor hardware details       
#define BOARD_ACCEL_I2C_ADDR  (0x1f) //I2C address (0x6A Adafruit 4517; 0x1F Adafruit 3643)
#define BOARD_MAG_I2C_ADDR    (0x1f) //I2C address (0x1C Adafruit 4517; 0x1F Adafruit 3643)
#define BOARD_GYRO_I2C_ADDR   (0x21) //I2C address (0x6A Adafruit 4517; 0x21 Adafruit 3643)
#define BOARD_THERM_I2C_ADDR  (0x1f) //I2C address (0x6A Adafruit 4517; 0x1F Adafruit 3643)

//pin that can be twiddled for debugging
#ifdef ESP8266
  //ESP8266 has different nomenclature for its GPIO pins and directions
  #define DEBUG_OUTPUT_PIN 15
  #define GPIO_MODE_OUTPUT OUTPUT
#endif
#ifdef ESP32
  //was GPIO_NUM_22 for esp_wrover and esp32dev boards, but not avail on nano.
  #define DEBUG_OUTPUT_PIN GPIO_NUM_14  
#endif

/**
 * Data can be output via a WiFi connection in addition to
 * (or instead of) a serial UART. The ESP device will act as
 * a WiFi Access Point (AP) with the credentials listed below.
 */
#if F_USE_WIRELESS_UART
  const char *ssid = "compass";
  const char *password = "northsouth";
  #define WIFI_STREAMING_PORT 23
  WiFiServer server(WIFI_STREAMING_PORT);  // use wifi server port 23 (telnet)
  WiFiClient tcp_client;
#endif

// Pointer to the Sensor Fusion object, created in setup() and used in loop()
SensorFusion *sensor_fusion;

// Variables used for timing the fusion calls and outputting data
unsigned long last_loop_time;
unsigned long last_print_time;

// Buffer for holding general text output
#define MAX_LEN_OUT_BUF 180
char output_str[MAX_LEN_OUT_BUF];

// Loop counter, used for toggling the debugging GPIO pin
int i;

void setup() {
  // put your setup code here, to run once:

  pinMode(DEBUG_OUTPUT_PIN, GPIO_MODE_OUTPUT);

  esp_log_level_set("*", ESP_LOG_DEBUG);  //inits serial port, sets up logging
  ESP_LOGI("setup()","Serial port configured.");

  // wifi config - using ESP as Access Point (AP)
#if F_USE_WIRELESS_UART
  // init WiFi connection
  WiFi.softAP(ssid, password);
  IPAddress myIP = WiFi.softAPIP();
  Serial.print("My AP IP address: ");
  Serial.println(myIP);
  server.begin(23);
  Serial.print("TCP server started. Connect to ");
  Serial.print(myIP);
  Serial.println(" on port 23.");
#endif

  //create our fusion engine instance
  sensor_fusion = new SensorFusion();

  /**
   * Fusion system can output arbitrary data & text, and accept incoming
   * commands These happen over either a hardware serial link, or a WiFi TCP
   * link. If this capability is desired then call
   * InitializeInputOutputSubsystem() with pointer to a serial stream (e.g.
   * &Serial) that has been initialized earlier in main (>= 115200 baud
   * recommended). Output can also be directed to a TCP client connecting on
   * port 23 - if this is desired, second parameter to
   * InitializeInputOutputSubsystem() is the pointer to the WiFiClient object.
   * Pass a NULL for stream(s) that aren't needed, or just call
   * InitializeInputOutputSubsystem() with no parameters. Note: to use the
   * streams, the appropriate #define F_USE_WIRELESS_UART and/or #define
   * F_USE_WIRED_UART has to be set in build.h The actual content sent on the
   * streams is determined by calling ProduceToolboxOutput() for Toolbox
   * compatible data packets, or SendArbitraryOutput() for whatever you have
   * placed in the Tx buffer. Using both calls is not recommended, as
   * interpreting the Toolbox data amongst your data will be confusing.
   */
#if F_USE_WIRELESS_UART && F_USE_WIRED_UART
  // setup IO subsystem to use both Serial and WiFi
  if (!(sensor_fusion->InitializeInputOutputSubsystem(&Serial, &tcp_client))) {
    ESP_LOGE("setup()","trouble initting Output and Control system: wireless and wired UART");
  }
#elif F_USE_WIRED_UART
  // setup IO subsystem to use only Serial port
  if (!(sensor_fusion->InitializeInputOutputSubsystem(&Serial, NULL))) {
    ESP_LOGE("setup()","trouble initting Output and Control system: wired UART");
  }
#elif F_USE_WIRELESS_UART
  // setup IO subsystem to use only WiFi
  if (!(sensor_fusion->InitializeInputOutputSubsystem(NULL, &tcp_client))) {
    ESP_LOGE("setup()","trouble initting Output and Control system: wireless UART");
  }
#else
  // setup IO subsystem for no output
  if (!(sensor_fusion->InitializeInputOutputSubsystem(NULL, NULL))) {
    ESP_LOGE("setup()","trouble initting Output and Control system: zero output");
  }
#endif

  // connect to the sensors.  Accelerometer and magnetometer are in same IC.
  if(! sensor_fusion->InstallSensor(BOARD_MAG_I2C_ADDR,
                               SensorType::kMagnetometer) ) {
    ESP_LOGE("setup()","trouble installing Magnetometer");
  }
  if(! sensor_fusion->InstallSensor(BOARD_ACCEL_I2C_ADDR,
                               SensorType::kAccelerometer) ) {
    ESP_LOGE("setup()","trouble installing Accelerometer");
  }
  if(! sensor_fusion->InstallSensor(BOARD_THERM_I2C_ADDR,
                               SensorType::kThermometer) ) {
    ESP_LOGE("setup()","trouble installing Thermometer");
  }
  if(! sensor_fusion->InstallSensor(BOARD_GYRO_I2C_ADDR,
                               SensorType::kGyroscope) ) {
    ESP_LOGE("setup()","trouble installing Gyroscope");
  }
  ESP_LOGI("setup()","Sensors connected");

  sensor_fusion->Begin(PIN_I2C_SDA, PIN_I2C_SCL);
  if(sensor_fusion->GetSystemStatus() == NORMAL)
  { ESP_LOGI("setup()","Fusion Engine Ready");
  }else
  { ESP_LOGW("setup()","Fusion status: %d\n",(int)sensor_fusion->GetSystemStatus());
    //may not see this if Begin() hangs, which it does when non-I2C pins chosen.
    //If pins are I2C-capable, but no physical sensor attached, then will see this error.
  }
  last_loop_time = millis(); //these will be used in loop()
  last_print_time = millis();

} // end setup()


void loop() {
  // put your main code here, to run repeatedly:

  /**
   * Usually the fusion algorithm is run once each time the sensors are read
   * However we allow for possibility of reading several times before fusing
   * (for instance, if we want to collect several magnetometer readings each
   * fusion cycle, since the magnetometer IC doesn't have a FIFO). To take
   * advantage of this feature, adjust the constants like kLoopsPerMagRead
   * in sensor_fusion_class.h
   */
  const unsigned long kLoopIntervalMs = 1000 / LOOP_RATE_HZ;
  const unsigned long kPrintIntervalMs = 1000;

#if F_USE_WIRELESS_UART
  if (!tcp_client) {
    tcp_client = server.available();  // check for incoming TCP clients
    if (tcp_client) {
      sensor_fusion->UpdateWiFiStream(&tcp_client);  
    }
  }
#endif 
  if ((millis() - last_loop_time) > kLoopIntervalMs) {
    last_loop_time += kLoopIntervalMs; //set up for next time through loop
    
    //read the Sensors whose turn it is, according to the loops_per_fuse_counter_ 
    sensor_fusion->ReadSensors(); 

    // run fusion routine according to loops_per_fuse_counter_
    sensor_fusion->RunFusion();

    //create and send Toolbox format packet if fusion has produced new data
    //This call is optional - if you don't want Toolbox packets, omit it
//    sensor_fusion->ProduceToolboxOutput();

    //Process any incoming commands arriving over serial or TCP port.
    //See control_input.c for list of available commands.
    //This call is optional - if you don't need external control, omit it
//    sensor_fusion->ProcessCommands();

//    sfg.applyPerturbation(
//            &sfg);  // apply debug perturbation (if testing mode enabled)
              //      Serial.println("applied perturbation");

    digitalWrite(DEBUG_OUTPUT_PIN, i % 2);  // toggle pin for debugging
    i++;

  }  // end of if() that reads sensors and runs fusion as needed

  // Send example output to Serial port
  // A few example parameters are chosen - see sensor_fusion_class.h for
  // a complete list of Get___() methods.
  if ((millis() - last_print_time) > kPrintIntervalMs) {
    last_print_time += kPrintIntervalMs;
    snprintf(output_str, MAX_LEN_OUT_BUF,
            "Heading %03.0f, Pitch %+4.0f, Roll %+4.0f, Temp %3.0fC, TurnRate %+5.0f, B %3.0f uT, Inc %3.0f deg, Status %d",
            sensor_fusion->GetHeadingDegrees(),
            sensor_fusion->GetPitchDegrees(),
            sensor_fusion->GetRollDegrees(),
            sensor_fusion->GetTemperatureC(),
            sensor_fusion->GetTurnRateDegPerS(),
            sensor_fusion->GetMagneticBMag(),
            sensor_fusion->GetMagneticInclinationDeg(),
            sensor_fusion->GetSystemStatus()
   );

    ESP_LOGI("main()", "%s", output_str ); //simplest way to see library output

    /**
     * If preferred, the library's input/output subsystem can be used
     * yo output data. This is useful, for example, to send the data
     * via WiFi instead of a wired connection. To do this, comment out
     * the Serial.print() command above, and use the 
     * following call to SendArbitraryData(), or the earlier-mentioned
     * ProduceToolboxOutput().  With either I/O subsystem call, ensure
     * that at least one of either the Serial stream or WiFi stream is
     * enabled in the file build.h (via F_USE_WIRELESS_UART and F_USE_WIRED_UART)
     * and that the call to InitializeInputOutputSubsystem() in setup()
     * references the desired stream(s).
     */
//    if (!sensor_fusion->SendArbitraryData(output_str, strlen(output_str))) {
//    Serial.println("couldn't send output");
//    }

  } // end timed if() that prints data as text
}  // end loop()

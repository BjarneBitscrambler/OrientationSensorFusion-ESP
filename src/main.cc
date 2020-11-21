#include <Arduino.h>
#include <sstream>
#include <string>
#ifdef ESP8266
  #include <ESP8266WiFi.h>
  #include <ESP8266WiFiAP.h>
#endif
#ifdef ESP32
  #include <WiFi.h>
  #include <WiFiAP.h>
#endif
#include <WiFiClient.h>
#include <Wire.h>

// Sensor Fusion Headers
#include "sensor_fusion_class.h"
#include "sensor_fusion.h"      // top level magCal and sensor fusion interfaces. Include 1st.
#include "board.h"              // hardware-specific settings. Edit as needed for board & sensors.
#include "build.h"
#include "control.h"  	        // Command processing and data streaming interface
#include "debug_print.h"        // provides ability to output debug messages via serial
#include "driver_sensors.h"     // hardware-specific drivers
#include "hal_i2c.h"            //I2C interfaces for ESP platform
#include "status.h"   	        // Status indicator interface - application specific

// wifi config - using ESP as Access Point (AP)
const char *ssid = "compass";
const char *password = "northsouth";
#define WIFI_STREAMING_PORT 23
WiFiServer server(WIFI_STREAMING_PORT);  // use wifi server port 23 (telnet)
WiFiClient client;  // TODO remove as global - is used in control.cpp

//pin that can be twiddled for debugging
#ifdef ESP8266
  //ESP8266 has different nomenclature for its GPIO pins and directions
  #define DEBUG_OUTPUT_PIN 13
  #define GPIO_MODE_OUTPUT OUTPUT
#endif
#ifdef ESP32
  #define DEBUG_OUTPUT_PIN GPIO_NUM_22  
#endif

// Sensor Fusion Global data structures
//SensorFusionGlobals sfg;                  //Primary sensor fusion data structure
struct ControlSubsystem controlSubsystem; // provides serial communications
struct StatusSubsystem statusSubsystem;   // provides visual (usually LED) status indicator
struct PhysicalSensor sensors[3];         // this implementation uses up to 3 sensors
SensorFusion *sensor_fusion;

void setup() {
  // put your setup code here, to run once:

    pinMode(DEBUG_OUTPUT_PIN, GPIO_MODE_OUTPUT);

    Serial.begin(BOARD_DEBUG_UART_BAUDRATE); //initialize serial UART
    delay(200);

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

  debug_log("waitasec...");  
  //delay not really necessary - gives time to open a serial monitor
  delay(1000);

  //initialize the I2C system at max clock rate supported by sensors
  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
  Wire.setClock( 400000 ); //in ESP8266 library, can't set clock in same call that sets pins
  debug_log("I2C initted");

  sensor_fusion = new SensorFusion(PIN_I2C_SDA, PIN_I2C_SCL);

  // initialize Sensor Fusion
/*  initializeControlPort(&controlSubsystem);  // configure pins and ports for the
                                             // control sub-system
  debug_log("Control Port OK");
  initializeStatusSubsystem(
      &statusSubsystem);  // configure pins and ports for the status sub-system
  debug_log("Status Subsystem OK");
  initSensorFusionGlobals(
      &sfg, &statusSubsystem,
      &controlSubsystem);  // Initialize sensor fusion structures
  debug_log("SFG OK");
*/

// connect to the sensors we will be using.  Accelerometer and magnetometer are in same IC.
  if(! sensor_fusion->InstallSensor(BOARD_ACCEL_MAG_I2C_ADDR,
                               SensorType::kMagnetometer) ) {
    debug_log("trouble installing Magnetometer");
  }
  if(! sensor_fusion->InstallSensor(BOARD_ACCEL_MAG_I2C_ADDR,
                               SensorType::kAccelerometer) ) {
    debug_log("trouble installing Accelerometer");
  }
  if(! sensor_fusion->InstallSensor(BOARD_GYRO_I2C_ADDR,
                               SensorType::kGyroscope) ) {
    debug_log("trouble installing Gyroscope");
  }

/*#if F_USING_ACCEL || F_USING_MAG
  sfg.installSensor(&sfg, &sensors[0], BOARD_ACCEL_MAG_I2C_ADDR, 1, NULL,
                    FXOS8700_Init, FXOS8700_Read);
  debug_log("Accel/Mag connected");
#endif
#if F_USING_GYRO
  sfg.installSensor(&sfg, &sensors[1], BOARD_GYRO_I2C_ADDR, 1, NULL,
                    FXAS21002_Init, FXAS21002_Read);
  debug_log("Gyro connected");
#endif
*/
/*  sfg.initializeFusionEngine(
      &sfg);  // Initialize sensors and magnetic calibration
  debug_log("Fusion Engine OK");

  sfg.setStatus(&sfg, NORMAL);  // Set status state to NORMAL
  debug_log("Passing to main...");
*/
  sensor_fusion->InitializeFusionEngine();
  debug_log("Fusion Engine OK");


} // end setup()

void loop() {
  // put your main code here, to run repeatedly:
    unsigned long last_call = millis();
    unsigned long last_print = millis();
    const unsigned long kLoopIntervalMs = 1000 / SENSOR_READ_RATE_HZ;
    const unsigned int kLoopsPerFusion = (1000 / FUSION_HZ) / kLoopIntervalMs;
    uint8_t loop_count = 0;
    while (true) {
#if F_USE_WIRELESS_UART
      if (!client) {
        client = server.available();  // listen for incoming TCP clients
        if (client) {
          //Serial.print("New Client on ");   
          //Serial.println(client.localIP());
        }
      }
#endif
      if ((millis() - last_call) > kLoopIntervalMs) {
        last_call += kLoopIntervalMs; //keep executing the below periodically
        
        sensor_fusion->ReadSensors(); //read the Sensors whose turn it is

        ++loop_count;
        if( loop_count == kLoopsPerFusion ) {
          loop_count = 0;
          sensor_fusion->RunFusion();  // run the fusion routines every 25 ms

        }
 
        if ((millis() - last_print) > 1000) {
          last_print += 1000;
          Serial.printf("%lu,%f\n\r", millis(),sensor_fusion->GetHeadingDegrees());
          //sensor_fusion->RunControlAndOutput();

        }

        /*      sfg.readSensors(
                  &sfg, (uint16_t)sfg.loopcounter);  // Reads sensors, applies
        HAL and                                               // does averaging
        (if applicable)
        //      debug_log("read sensors");
              sfg.conditionSensorReadings(&sfg);  // magCal (magnetic
        calibration) is part of this
        //      debug_log("applied cal");
              digitalWrite(
                  DEBUG_OUTPUT_PIN, 0);  // toggle output pin each time through,
        for debugging sfg.runFusion(&sfg);                // Run the actual
        fusion algorithms digitalWrite( DEBUG_OUTPUT_PIN, 1);  // toggle output
        pin each time through, for debugging
        //      debug_log("fused");
              // Serial.printf(" Algo took %ld us\n",
        sfg.SV_9DOF_GBY_KALMAN.systick);
      */
//        sfg.applyPerturbation(
//            &sfg);  // apply debug perturbation (if testing mode enabled)
                    //      debug_log("applied perturbation");

        //Make and send data to the Sensor Fusion Toolbox or whatever UART is connected to.
//        sfg.pControlSubsystem->stream(&sfg);  //create the output packet  
//        sfg.pControlSubsystem->write(sfg.pControlSubsystem);  //send the output packet

//        sfg.pControlSubsystem->readCommands(&sfg);
//        digitalWrite(
//            DEBUG_OUTPUT_PIN, i % 2);  // toggle output pin each time through, for debugging
      }//end of loop that reads sensors and runs fusion as needed
    }//end while(true)
} // end loop()

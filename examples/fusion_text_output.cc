#include <Arduino.h>
#include <sstream>
#include <string>
#include <Wire.h>
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
#include "sensor_fusion/sensor_fusion.h"      // top level magCal and sensor fusion interfaces. Include 1st.
#include "board.h"              // hardware-specific settings. Edit as needed for board & sensors.
#include "build.h"
#include "sensor_fusion/control.h"  	        // Command processing and data streaming interface
#include "sensor_fusion/debug_print.h"        // provides ability to output debug messages via serial
#include "sensor_fusion/driver_sensors.h"     // hardware-specific drivers
#include "sensor_fusion/hal_i2c.h"            //I2C interfaces for ESP platform
#include "sensor_fusion/status.h"   	        // Status indicator interface - application specific


//pin that can be twiddled for debugging
#ifdef ESP8266
  //ESP8266 has different nomenclature for its GPIO pins and directions
  #define DEBUG_OUTPUT_PIN 13
  #define GPIO_MODE_OUTPUT OUTPUT
#endif
#ifdef ESP32
  #define DEBUG_OUTPUT_PIN GPIO_NUM_22  
#endif

#if F_USE_WIRELESS_UART
  const char *ssid = "compass";
  const char *password = "northsouth";
  #define WIFI_STREAMING_PORT 23
  WiFiServer server(WIFI_STREAMING_PORT);  // use wifi server port 23 (telnet)
  WiFiClient tcp_client;
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

  debug_log("waitasec...");  
  //delay not necessary - gives time to open a serial monitor
  delay(1000);

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
#else
  Serial *tcp_client = NULL;
#endif

  //initialize the I2C system at max clock rate supported by sensors
  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
  Wire.setClock( 400000 ); //in ESP8266 library, can't set clock in same call that sets pins
  debug_log("I2C initted");

  //create our fusion engine instance
  sensor_fusion = new SensorFusion(PIN_I2C_SDA, PIN_I2C_SCL);

  //Fusion system creates a data packet of sensor readings and algorithm results
  //formatted for the NXP Sensor Toolbox program. If this is desired then call
  //InitializeControlSubsystem() with pointer to a serial stream (e.g. &Serial)
  //that has been initialized earlier in main (>115200 baud recommended).
  //Output can also be directed to a TCP client connecting on port 23 - if this
  //is desired, second parameter to InitializeControlSubsystem() is the 
  //pointer to the WiFiClient object.  Pass a NULL for stream(s)
  //aren't needed, or just call InitializeControlSubsystem() with no parameters.
  //Note: to use the streams, the appropriate #define has to be set in build.h  
//  if( ! sensor_fusion->InitializeControlSubsystem(&Serial, &tcp_client) ) {
  if( ! sensor_fusion->InitializeControlSubsystem(NULL, &tcp_client) ) {
    debug_log("trouble initting Output and Control system");
  }

  // connect to the sensors.  Accelerometer and magnetometer are in same IC.
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
  debug_log("Sensors connected");

  sensor_fusion->InitializeFusionEngine();
  debug_log("Fusion Engine OK");

} // end setup()

void loop() {
  // put your main code here, to run repeatedly:

    //Usually the fusion algorithm is run once each time the sensors are read
    //However we allow for possibility of reading several times before fusing
    //(for instance, if we want to collect several magnetometer readings each
    //fusion cycle, since the magnetometer IC doesn't have a FIFO). To take
    //advantage of this feature, adjust the constants like kLoopsPerMagRead
    //in sensor_fusion_class.h
    const unsigned long kLoopIntervalMs = 1000 / LOOP_RATE_HZ;
    const unsigned long kPrintIntervalMs = 250;

    unsigned long last_loop_time = millis();
    unsigned long last_print_time = millis();

    while (true) {
#if F_USE_WIRELESS_UART
      if (!tcp_client) {
        tcp_client = server.available();  // listen for incoming TCP clients
        if (tcp_client) {
          sensor_fusion->UpdateWiFiStream(&tcp_client);  
          // Serial.print("New Client on ");
          // Serial.println(client.localIP());
        }
      }
#endif
      if ((millis() - last_loop_time) > kLoopIntervalMs) {
        last_loop_time += kLoopIntervalMs; //keep executing the below periodically
        
        //read the Sensors whose turn it is, according to the loops_per_fuse_counter_ 
        sensor_fusion->ReadSensors(); 

        // run fusion routine according to loops_per_fuse_counter_
        sensor_fusion->RunFusion();

        //create and send output if fusion has produced new data
        sensor_fusion->ProduceToolboxOutput();

        //process any incoming commands
        sensor_fusion->ProcessCommands();
        
        if ((millis() - last_print_time) > kPrintIntervalMs) {
          last_print_time += kPrintIntervalMs;
          Serial.printf("%lu,\t%03.1f, %+5.1f, %+5.1f,\t%+4.0f, %+4.0f, %+4.0f,\t%+5.2f, %+5.2f, %+5.2f\n\r", millis(),
                        sensor_fusion->GetHeadingDegrees(),
                        sensor_fusion->GetPitchDegrees(),
                        sensor_fusion->GetRollDegrees(),
                        sensor_fusion->GetTurnRateDegPerS(),
                        sensor_fusion->GetPitchRateDegPerS(),
                        sensor_fusion->GetRollRateDegPerS(),
                        sensor_fusion->GetAccelXGees(),
                        sensor_fusion->GetAccelYGees(),
                        sensor_fusion->GetAccelZGees()
                        );
        }

//        sfg.applyPerturbation(
//            &sfg);  // apply debug perturbation (if testing mode enabled)
                    //      debug_log("applied perturbation");

//        digitalWrite(DEBUG_OUTPUT_PIN, i % 2);  // toggle pin for debugging

      }//end of loop that reads sensors and runs fusion as needed
    }//end while(true)
} // end loop()

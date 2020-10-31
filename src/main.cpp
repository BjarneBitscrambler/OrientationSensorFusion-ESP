#include <Arduino.h>
#include <Wire.h>

#include <sstream>
#include <string>

// Sensor Fusion Headers
#include "sensor_fusion.h"      // top level magCal and sensor fusion interfaces
#include "control.h"  	        // Command/Streaming interface - application specific
#include "status.h"   	        // Status indicator interface - application specific
#include "drivers.h"  	        // NXP sensor drivers OR customer-supplied drivers


#include "board.h"

#include "debug_print.h"

#include "sensor_io_i2c_sensesp.h"
 
// Sensor Fusion Global data structures
SensorFusionGlobals sfg;                ///< This is the primary sensor fusion data structure
struct ControlSubsystem controlSubsystem;      ///< used for serial communications
struct StatusSubsystem statusSubsystem;        ///< provides visual (usually LED) status indicator
struct PhysicalSensor sensors[3];              ///< This implementation uses up to 3 sensors
//uint8_t           testBuffer[256]; 

void setup() {
  // put your setup code here, to run once:
// init rs port
    Serial.begin(115200);
    delay(200);
//    Debug.setSerialEnabled(true);
//    delay(3000);
//    debugI("Serial debug enabled");

   // init wifi connection
/*   if (!WiFi.config(local_IP, gateway, subnet, primaryDNS, secondaryDNS)) {
     debug_log("Failed to configure network settings");
    }
   WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        debug_log("connecting to WiFi network");
        delay(500);
    }
   debug_log("connected to WiFi");
   debug_log("IP adddr: ");
   Serial.println(WiFi.localIP());
*/
   debug_log("waitasec...");
   delay(1000);

  //initialize the I2C system (hack - using other AHRS)
 Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL, 400000 );
   debug_log("I2C initted");

/*   byte incoming[4];
   while (true) {
     I2CReadByte(0x1f, 0x0d, &(incoming[0]));
     delay(1);
     I2CReadBytes(0x1f, 0x0c, &(incoming[0]),3);
     delay(100);
   }
*/
  // initialize Sensor Fusion
  initializeControlPort(&controlSubsystem);  // configure pins and ports for the
                                             // control sub-system
  debug_log("Control Port OK");
  initializeStatusSubsystem(
      &statusSubsystem);  // configure pins and ports for the status sub-system
  debug_log("Status Subsystem OK");
  initSensorFusionGlobals(
      &sfg, &statusSubsystem,
      &controlSubsystem);  // Initialize sensor fusion structures
  debug_log("SFG OK");

// "install" the sensors we will be using
#if F_USING_ACCEL || F_USING_MAG
    sfg.installSensor(&sfg, &sensors[0], FXOS8700_ADDRESS, 1, NULL, NULL, FXOS8700_Init,  FXOS8700_Read);
   debug_log("Accel/Mag connected");
#endif
#if F_USING_GYRO
    sfg.installSensor(&sfg, &sensors[1], FXAS21002C_ADDRESS, 1, NULL, NULL, FXAS21002_Init, FXAS21002_Read);
   debug_log("Gyro connected");
#endif

   sfg.initializeFusionEngine(&sfg);	        // This will initialize sensors and magnetic calibration
   debug_log("Fusion Engine OK");

    sfg.setStatus(&sfg, NORMAL);                // If we got this far, let's set status state to NORMAL
   debug_log("Passing to main...");
   delay(1000);
}

void loop() {
  // put your main code here, to run repeatedly:
    int last_call = millis();
    int loop_interval_ms = 1000 / FUSION_HZ;
    int i = 0;
    while (true) {
      if ((millis() - last_call) > loop_interval_ms) {
        last_call += loop_interval_ms;
        
//        FXAS21002_Read(&sensors[1],&sfg);

        sfg.readSensors(&sfg, (uint16_t)sfg.loopcounter);  // Reads sensors, applies HAL and does
                                   // averaging (if applicable)
//   debug_log("read sensors");
        sfg.conditionSensorReadings(&sfg);  // magCal is run as part of this
//   debug_log("applied cal");
        sfg.runFusion(&sfg);                // Run the actual fusion algorithms
//   debug_log("fused");
        sfg.applyPerturbation(&sfg);  // apply debug perturbation (testing only)
//   debug_log("applied perturbation");
        sfg.loopcounter++;  // The loop counter is used to "serialize" mag cal
                            // operations
        i = i + 1;
        if (i >=
            4) {  // Some status codes include a "blink" feature.  This loop
          i = 0;  // should cycle at least four times for that to operate
                  // correctly.
          sfg.updateStatus(
              &sfg);  // This is where pending status updates are made visible
//   debug_log("updated status");
        }

        sfg.queueStatus(
            &sfg,
            NORMAL);  // assume NORMAL status for next pass through the loop
//               debug_log("entering stream...");

        sfg.pControlSubsystem->stream(
            &sfg, sUARTOutputBuffer);  //was sUARTOutputBuffer Send stream data to the Sensor Fusion
                                       // Toolbox
      }
    }
    
}

/*
 * Copyright (c) 2015, Freescale Semiconductor, Inc.
 * Copyright 2016-2018 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef _BOARD_H_
#define _BOARD_H_

#if defined(__cplusplus)
extern "C" {
#endif /* __cplusplus */

#include "esp32-hal-gpio.h" //may have to replace if using ESP8266. Needed for pinMode() etc.
//#include "Arduino.h" //Can use this instead (which includes the *_hal_gpio), but then some other
// constants get defined too (like PI) which clash with defines in sensor_fusion.h

/*******************************************************************************
 * Definitions
 ******************************************************************************/
/*! @brief The board name, passed in packets to Sensor Toolbox. Don't know 
 whether they are important - suspect they are only informational. */
#define BOARD_NAME "ESP32 WROVER"
#define THIS_BOARD  9   //impersonates a FRDM_K22F. Sent in packets to PC-based App.
#define THIS_SHIELD 4   //impersonates shield AGMP03. Sent in packets to PC-based App.

#define PIN_I2C_SDA   (23)  //Adjust to your board. A value of -1
#define PIN_I2C_SCL   (25)  // will use default Arduino pins.

/*! @brief The UART to use for data streaming and debug messages. */
#ifndef BOARD_DEBUG_UART_BAUDRATE
#define BOARD_DEBUG_UART_BAUDRATE 115200
#endif

//sensor hardware details
#define FXAS21002C_I2C_ADDRESS      (0x21) //I2C address on Adafruit breakout board
#define FXOS8700_I2C_ADDRESS        (0x1F) //I2C address on Adafruit breakout board
#define BOARD_ACCEL_MAG_I2C_ADDR    FXOS8700_I2C_ADDRESS
#define BOARD_GYRO_I2C_ADDR         FXAS21002C_I2C_ADDRESS

// Board LED mappings for ESP32 WROVER-KIT
#define LOGIC_LED_ON  1U
#define LOGIC_LED_OFF 0U

#ifndef BOARD_LED_RED_GPIO_PIN
#define BOARD_LED_RED_GPIO_PIN (0)
#endif
#ifndef BOARD_LED_GREEN_GPIO_PIN
#define BOARD_LED_GREEN_GPIO_PIN (2)
#endif
#ifndef BOARD_LED_BLUE_GPIO_PIN
#define BOARD_LED_BLUE_GPIO_PIN (4)
#endif

#ifndef LED_BUILTIN
#define LED_BUILTIN BOARD_LED_RED_GPIO_PIN
#endif

//the following LED-related macros replace the functions used in status.c
#define LED_RED_INIT(output)                                           \
    pinMode(BOARD_LED_RED_GPIO_PIN, OUTPUT);  // Enable LED_RED
#define LED_RED_ON()  digitalWrite(BOARD_LED_RED_GPIO_PIN, HIGH); // Turn on LED_RED 
#define LED_RED_OFF() digitalWrite(BOARD_LED_RED_GPIO_PIN, LOW); // Turn off LED_RED 
#define LED_RED_TOGGLE() \
    digitalWrite(BOARD_LED_RED_GPIO_PIN, !digitalRead(BOARD_LED_RED_GPIO_PIN)); // Toggle LED_RED

#define LED_GREEN_INIT(output)                                           \
    pinMode(BOARD_LED_GREEN_GPIO_PIN, OUTPUT);  // Enable LED_GREEN
#define LED_GREEN_ON()  digitalWrite(BOARD_LED_GREEN_GPIO_PIN, HIGH); // Turn on LED_GREEN 
#define LED_GREEN_OFF() digitalWrite(BOARD_LED_GREEN_GPIO_PIN, LOW); // Turn off LED_GREEN 
#define LED_GREEN_TOGGLE() \
    digitalWrite(BOARD_LED_GREEN_GPIO_PIN, !digitalRead(BOARD_LED_GREEN_GPIO_PIN)); // Toggle LED_GREEN

#define LED_BLUE_INIT(output)                                           \
    pinMode(BOARD_LED_BLUE_GPIO_PIN, OUTPUT);  // Enable LED_BLUE
#define LED_BLUE_ON()  digitalWrite(BOARD_LED_BLUE_GPIO_PIN, HIGH); // Turn on LED_BLUE 
#define LED_BLUE_OFF() digitalWrite(BOARD_LED_BLUE_GPIO_PIN, LOW); // Turn off LED_BLUE 
#define LED_BLUE_TOGGLE() \
    digitalWrite(BOARD_LED_BLUE_GPIO_PIN, !digitalRead(BOARD_LED_BLUE_GPIO_PIN)); // Toggle LED_BLUE


#define CLOCK_EnableClock(x)        //found in status.c.  We don't need it.
#define PORT_SetPinMux(x,y,z)       //found in status.c   We don't need it.


/*******************************************************************************
 * API
 ******************************************************************************/

void BOARD_InitDebugConsole(void);

#if defined(__cplusplus)
}
#endif /* __cplusplus */

#endif /* _BOARD_H_ */

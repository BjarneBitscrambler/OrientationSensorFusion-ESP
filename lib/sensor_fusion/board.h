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

//#include "clock_config.h"
//#include "fsl_gpio.h"
#include "esp32-hal-gpio.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
/*! @brief The board name */
#define BOARD_NAME "ESP32 WROVER"
#define THIS_BOARD  9   //impersonates a FRDM_K22F. Sent in packets to PC-based App.
#define THIS_SHIELD 4   //impersonates shield AGMP03. Sent in packets to PC-based App.

#define PIN_I2C_SDA   (23)  //Adjust to your board. A value of -1
#define PIN_I2C_SCL   (25)  // will use default Arduino pins.

/*! @brief The UART to use for debug messages. */
#define BOARD_USE_UART
#define BOARD_DEBUG_UART_TYPE     kSerialPort_Uart
#define BOARD_DEBUG_UART_BASEADDR (uint32_t) UART1
#define BOARD_DEBUG_UART_INSTANCE 1U
#define BOARD_DEBUG_UART_CLKSRC   SYS_CLK
//#define BOARD_DEBUG_UART_CLK_FREQ CLOCK_GetCoreSysClkFreq()
//#define BOARD_UART_IRQ            UART1_RX_TX_IRQn
//#define BOARD_UART_IRQ_HANDLER    UART1_RX_TX_IRQHandler

#ifndef BOARD_DEBUG_UART_BAUDRATE
#define BOARD_DEBUG_UART_BAUDRATE 115200
#endif /* BOARD_DEBUG_UART_BAUDRATE */

//sensor hardware details
#define FXAS21002C_ADDRESS (0x21) //I2C address on Adafruit breakout board
#define FXOS8700_ADDRESS (0x1F) //I2C address on Adafruit breakout board
#define BOARD_FXOS8700_ADDR        FXOS8700_ADDRESS
/* Board accelerometer driver */
#define BOARD_ACCEL_FXOS
#define BOARD_ACCEL_ADDR           BOARD_FXOS8700_ADDR
#define BOARD_ACCEL_BAUDRATE       100

//#define BOARD_ACCEL_I2C_BASEADDR   I2C0
//#define BOARD_ACCEL_I2C_CLOCK_FREQ CLOCK_GetFreq(I2C0_CLK_SRC)

/*! @brief The i2c instance used for i2c connection by default */
//#define BOARD_I2C_BASEADDR I2C0

/*! @brief The CMP instance/channel used for board. */
//#define BOARD_CMP_BASEADDR CMP0
//#define BOARD_CMP_CHANNEL  0U

/*! @brief The rtc instance used for board. */
//#define BOARD_RTC_FUNC_BASEADDR RTC

/*! @brief Define the port interrupt number for the board switches */
/*#ifndef BOARD_SW3_GPIO
#define BOARD_SW3_GPIO GPIOB
#endif
#ifndef BOARD_SW3_PORT
#define BOARD_SW3_PORT PORTB
#endif
#ifndef BOARD_SW3_GPIO_PIN
#define BOARD_SW3_GPIO_PIN 17
#endif
#define BOARD_SW3_IRQ         PORTB_IRQn
#define BOARD_SW3_IRQ_HANDLER PORTB_IRQHandler
#define BOARD_SW3_NAME        "SW3"

#ifndef BOARD_SW2_GPIO
#define BOARD_SW2_GPIO GPIOC
#endif
#ifndef BOARD_SW2_PORT
#define BOARD_SW2_PORT PORTC
#endif
#ifndef BOARD_SW2_GPIO_PIN
#define BOARD_SW2_GPIO_PIN 1
#endif
#define BOARD_SW2_IRQ         PORTC_IRQn
#define BOARD_SW2_IRQ_HANDLER PORTC_IRQHandler
#define BOARD_SW2_NAME        "SW2"

#define LLWU_SW_GPIO        BOARD_SW2_GPIO
#define LLWU_SW_PORT        BOARD_SW2_PORT
#define LLWU_SW_GPIO_PIN    BOARD_SW2_GPIO_PIN
#define LLWU_SW_IRQ         BOARD_SW2_IRQ
#define LLWU_SW_IRQ_HANDLER BOARD_SW2_IRQ_HANDLER
#define LLWU_SW_NAME        BOARD_SW2_NAME
*/

// Board led color mapping for WROVER-KIT
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

#define CLOCK_EnableClock(x)        //found in status.c
#define PORT_SetPinMux(x,y,z)       //found in status.c


//#define BOARD_ARDUINO_INT_IRQ   (PORTB_IRQn)
//#define BOARD_ARDUINO_I2C_IRQ   (I2C0_IRQn)
//#define BOARD_ARDUINO_I2C_INDEX (0)

/*******************************************************************************
 * API
 ******************************************************************************/

void BOARD_InitDebugConsole(void);
#if defined(SDK_I2C_BASED_COMPONENT_USED) && SDK_I2C_BASED_COMPONENT_USED
void BOARD_I2C_Init(I2C_Type *base, uint32_t clkSrc_Hz);
status_t BOARD_I2C_Send(I2C_Type *base,
                        uint8_t deviceAddress,
                        uint32_t subAddress,
                        uint8_t subaddressSize,
                        uint8_t *txBuff,
                        uint8_t txBuffSize);
status_t BOARD_I2C_Receive(I2C_Type *base,
                           uint8_t deviceAddress,
                           uint32_t subAddress,
                           uint8_t subaddressSize,
                           uint8_t *rxBuff,
                           uint8_t rxBuffSize);
void BOARD_Accel_I2C_Init(void);
status_t BOARD_Accel_I2C_Send(uint8_t deviceAddress, uint32_t subAddress, uint8_t subaddressSize, uint32_t txBuff);
status_t BOARD_Accel_I2C_Receive(
    uint8_t deviceAddress, uint32_t subAddress, uint8_t subaddressSize, uint8_t *rxBuff, uint8_t rxBuffSize);
#endif /* SDK_I2C_BASED_COMPONENT_USED */

#if defined(__cplusplus)
}
#endif /* __cplusplus */

#endif /* _BOARD_H_ */

/*
 Bjarne Hansen 2020-10-27 for ESP32 environment

 */

/*! \file replacement_functions.h
    \brief Wrapper for Hardware Abstraction Layer (HAL)

    Contains replacements for HAL_*() functions defined in drivers.h, so #include 
    this file after drivers.h
*/

#ifndef __REPLACEMENT_FUNCTIONS_H__
#define __REPLACEMENT_FUNCTIONS_H__

#include <Arduino.h>
#include <esp32-hal.h>  //is this really needed? Doesn't Arduino.h include this?
#include <stdint.h>


#ifdef __cplusplus
extern "C" {
#endif

 void ARM_systick_enable(void);
 void ARM_systick_start_ticks(int32_t *pstart);
 int32_t ARM_systick_elapsed_ticks(int32_t start_ticks);
 void ARM_systick_delay_ms(uint32_t iSystemCoreClock, uint32_t delay_ms);

#ifdef __cplusplus
}
#endif

#endif  // __REPLACEMENT_FUNCTIONS_H__

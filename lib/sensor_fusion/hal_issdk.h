/*
 * Copyright (c) 2015, Freescale Semiconductor, Inc.
 * Copyright 2016-2017 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause

modified Bjarne Hansen 2020-10-27 for ESP32 environment

 */

/*! \file issdk_hal.h
    \brief Wrapper for Hardware Abstraction Layer (HAL)
    Contains replacements for hardware-specific functions defined in 
    drivers.h, so #include this file after drivers.h
*/

#ifndef __HAL_ISSDK_H__
#define __HAL_ISSDK_H__

#ifdef __cplusplus
extern "C" {
#endif

#define CORE_SYSTICK_HZ 1000000     //since we know we have 1us resolution available on ESP processors

 void ARM_systick_enable(void);
 void ARM_systick_start_ticks(int32_t *pstart);
 int32_t ARM_systick_elapsed_ticks(int32_t start_ticks);
 void ARM_systick_delay_ms(uint32_t iSystemCoreClock, uint32_t delay_ms);

#ifdef __cplusplus
}
#endif

#endif  // __HAL_ISSDK_H__

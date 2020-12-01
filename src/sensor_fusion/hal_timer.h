/*
 * Copyright (c) 2015, Freescale Semiconductor, Inc.
 * Copyright (c) 2016-2017 NXP
 * Copyright (c) 2020 Bjarne Hansen
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause

 */

/*! \file hal_timer.h
    \brief Wrapper for Hardware Abstraction Layer (HAL)
    Contains replacements for hardware-specific functions 
    Currently only timer functions.
*/

#ifndef __HAL_TIMER_H__
#define __HAL_TIMER_H__

#ifdef __cplusplus
extern "C" {
#endif

 void SystickStartCount(int32_t *pstart);
 int32_t SystickElapsedMicros(int32_t start_ticks);
 void SystickDelayMillis(uint32_t delay_ms);

#ifdef __cplusplus
}
#endif

#endif  // __HAL_TIMER_H__

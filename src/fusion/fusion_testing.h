/*
 * Copyright (c) 2015, Freescale Semiconductor, Inc.
 * Copyright 2016-2017 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/*! \file fusion_testing.h
    \brief ApplyPerturbation function used to analyze dynamic performance

    The ApplyPerturbation function applies a user-specified step function to
    prior fusion results which is then "released" in the next fusion cycle.
    When used in conjustion with the NXP Sensor Fusion Toolbox, this provides
    a visual indication of the dynamic behavior of the library.
*/


#ifndef FUSION_TESTING_H
#define FUSION_TESTING_H

#ifdef __cplusplus
extern "C" {
#endif

// prototypes for functions defined in fusion_testing.c
void ApplyPerturbation(SensorFusionGlobals *sfg);


#ifdef __cplusplus
}
#endif

#endif // #ifndef FUSION_TESTING_H

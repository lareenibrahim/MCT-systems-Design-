/*
 * odommetry.h
 *
 *  Created on: May 30, 2026
 *      Author: Osama Mohammed
 */

#ifndef ODOMETRY_H
#define ODOMETRY_H

#include "stm32f4xx_hal.h"
#include "Base_Position.h"   // for CPM, WHEEL_BASE_X/Y, s32

typedef struct {
    float x;        // metres, world frame
    float y;        // metres, world frame
    float theta;    // radians, from fused heading — do NOT integrate here
} Pose_t;

void Odometry_Init  (Pose_t *pose);
void Odometry_Update(Pose_t *pose,
                     s32    enc_now[4],
                     s32    enc_prev[4],
                     float  fused_heading_rad,  // comes from HeadingFusion
                     float  dt_s);
#endif

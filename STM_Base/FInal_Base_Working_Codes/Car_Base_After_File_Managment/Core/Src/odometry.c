/*
 * odometry.c
 *
 *  Created on: May 30, 2026
 *      Author: Osama Mohammed
 */


#include "odometry.h"
#include <math.h>
#include <string.h>

/* Must match Base_Position.c — same sign correction */
static const int   ENC_SIGN[4] = { -1,  1,  1, -1 };
static const float CPM_VAL[4]  = {
    1974.7f / (3.14159f * 0.097f),
    1966.4f / (3.14159f * 0.097f),
    1975.0f / (3.14159f * 0.097f),
    1970.0f / (3.14159f * 0.097f),
};

void Odometry_Init(Pose_t *pose)
{
    pose->x     = 0.0f;
    pose->y     = 0.0f;
    pose->theta = 0.0f;
}

void Odometry_Update(Pose_t *pose,
                     s32    enc_now[4],
                     s32    enc_prev[4],
                     float  fused_heading_rad,
                     float  dt_s)
{
    /* ── Step 1: delta metres per wheel ── */
    float d[4];
    for (int i = 0; i < 4; i++)
        d[i] = (float)((enc_now[i] - enc_prev[i]) * ENC_SIGN[i])
               / CPM_VAL[i];

    /* ── Step 2: mecanum forward kinematics → body frame ──
     * Layout: d[0]=RR, d[1]=FR, d[2]=RL, d[3]=FL
     * Standard mecanum model:
     *   vx  = ( d0 + d1 + d2 + d3) / 4
     *   vy  = (-d0 + d1 + d2 - d3) / 4   (strafe: +Y = left)
     * θ is NOT computed from encoders — use fused heading instead
     */
    float dx_body = ( d[0] + d[1] + d[2] + d[3]) * 0.25f;
    float dy_body = (-d[0] + d[1] + d[2] - d[3]) * 0.25f;

    /* ── Step 3: rotate body displacement into world frame ── */
    float c = cosf(fused_heading_rad);
    float s = sinf(fused_heading_rad);

    pose->x    += dx_body * c - dy_body * s;
    pose->y    += dx_body * s + dy_body * c;
    pose->theta = fused_heading_rad;   // always from fusion, never integrated
}

/*
 * imu_fusion.c
 *
 *  Created on: May 30, 2026
 *      Author: Osama Mohammed
 */

#include "imu_fusion.h"

void HeadingFusion_Init(HeadingFusion_t *f, float alpha,
                         float initial_mag_heading)
{
    f->alpha       = alpha;
    f->heading_deg = initial_mag_heading; // seed with mag so first reading
                                          // isn't a jump from 0
}

float HeadingFusion_Update(HeadingFusion_t *f,
                            float gyro_z_dps,       // imu_data.gyro.z  (bias already removed)
                            float mag_heading_deg,  // mag.heading       (0–360, bias already removed)
                            float dt_s)             // 0.02f for 20 ms loop
{
    // ── Step 1: Integrate gyro ──────────────────────────────
    // NOTE: check your robot. If rotating CCW increases mag heading,
    // use +gyro_z. If it decreases, use -gyro_z.
    float gyro_heading = f->heading_deg - gyro_z_dps * dt_s;

    // ── Step 2: Wrap gyro_heading to [0, 360) ───────────────
    while (gyro_heading >= 360.0f) gyro_heading -= 360.0f;
    while (gyro_heading <    0.0f) gyro_heading += 360.0f;

    // ── Step 3: Find shortest angular difference ────────────
    // Prevents 359°→1° blending to 180° (the wraparound bug)
    float diff = mag_heading_deg - gyro_heading;
    while (diff >  180.0f) diff -= 360.0f;
    while (diff < -180.0f) diff += 360.0f;

    // ── Step 4: Complementary blend ─────────────────────────
    f->heading_deg = gyro_heading + (1.0f - f->alpha) * diff;

    // ── Step 5: Keep output in [0, 360) ─────────────────────
    while (f->heading_deg >= 360.0f) f->heading_deg -= 360.0f;
    while (f->heading_deg <    0.0f) f->heading_deg += 360.0f;

    return f->heading_deg;
}

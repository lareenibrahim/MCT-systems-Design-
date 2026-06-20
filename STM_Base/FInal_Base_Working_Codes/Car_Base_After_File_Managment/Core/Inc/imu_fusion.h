#ifndef FUSION_H
#define FUSION_H

#include "mpu6050.h"
#include "mag_sensor.h"

typedef struct {
    float heading_deg;  // fused heading output, 0–360
    float alpha;        // τ/(τ+dt), tune between 0.95–0.99
} HeadingFusion_t;

void  HeadingFusion_Init  (HeadingFusion_t *f, float alpha,
                            float initial_mag_heading);
float HeadingFusion_Update(HeadingFusion_t *f,
                            float gyro_z_dps,
                            float mag_heading_deg,
                            float dt_s);
#endif

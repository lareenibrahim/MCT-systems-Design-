#ifndef MAG_SENSOR_H
#define MAG_SENSOR_H

#include "stm32f4xx_hal.h"
#include <stdint.h>

/* ── Register map ───────────────────────────────────────────── */
#define MAG_ADDR         (0x2C << 1)
#define MAG_REG_DATA     0x01
#define MAG_REG_CTRL     0x0A
#define MAG_SENSITIVITY  0.15f

/* ── Handle ─────────────────────────────────────────────────── */
typedef struct
{
    I2C_HandleTypeDef *hi2c;

    /* Raw values */
    int16_t raw_x;
    int16_t raw_y;
    int16_t raw_z;

    /* Scaled values */
    float x_uT;
    float y_uT;
    float z_uT;

    /* Heading */
    float heading;

    /* Calibration biases */
    float bias_x;
    float bias_y;
    float bias_z;

} MAG_Handle;

/* ── Public Functions ───────────────────────────────────────── */
void              MAG_Init    (MAG_Handle *dev, I2C_HandleTypeDef *hi2c);
HAL_StatusTypeDef MAG_Read    (MAG_Handle *dev);
void              Mag_Calibrate(MAG_Handle *dev);

#endif /* MAG_SENSOR_H */

#include "mag_sensor.h"
#include <math.h>
#include <stdio.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

/* ── Init ───────────────────────────────────────────────────── */
void MAG_Init(MAG_Handle *dev, I2C_HandleTypeDef *hi2c)
{
    dev->hi2c = hi2c;

    /* Hardcoded calibration for this unit —
       replace these values after recalibrating */
    dev->bias_x = -14.4750f;
    dev->bias_y = -39.0000f;
    dev->bias_z = -0.0750f;

    dev->raw_x  = 0;
    dev->raw_y  = 0;
    dev->raw_z  = 0;
    dev->x_uT   = 0.0f;
    dev->y_uT   = 0.0f;
    dev->z_uT   = 0.0f;
    dev->heading = 0.0f;
}

/* ── MAG_Read ───────────────────────────────────────────────── */
HAL_StatusTypeDef MAG_Read(MAG_Handle *dev)
{
    uint8_t buf[6] = {0};
    uint8_t cmd    = 0x01;

    HAL_StatusTypeDef ret = HAL_I2C_Mem_Write(dev->hi2c, MAG_ADDR, MAG_REG_CTRL,
                                               1, &cmd, 1, 100);
    if (ret != HAL_OK) return ret;

    HAL_Delay(10);

    ret = HAL_I2C_Mem_Read(dev->hi2c, MAG_ADDR, MAG_REG_DATA, 1, buf, 6, 100);
    if (ret != HAL_OK) return ret;

    dev->raw_x = (int16_t)(buf[0] | (buf[1] << 8));
    dev->raw_y = (int16_t)(buf[2] | (buf[3] << 8));
    dev->raw_z = (int8_t)buf[4];

    dev->x_uT = (dev->raw_x * MAG_SENSITIVITY) - dev->bias_x;
    dev->y_uT = (dev->raw_y * MAG_SENSITIVITY) - dev->bias_y;
    dev->z_uT = (dev->raw_z * MAG_SENSITIVITY) - dev->bias_z;

    dev->heading = atan2f(dev->y_uT, dev->x_uT) * (180.0f / (float)M_PI);
    if (dev->heading < 0.0f) dev->heading += 360.0f;

    return HAL_OK;
}

/* ── Mag_Calibrate ──────────────────────────────────────────── */
void Mag_Calibrate(MAG_Handle *dev)
{
    printf("Mag calibration starting in 3 seconds...\r\n");
    printf("When collection starts, rotate the sensor slowly\r\n");
    printf("through ALL orientations for 30 seconds.\r\n");
    printf("Flat circles, tilted circles, on edge, upside down.\r\n");
    HAL_Delay(3000);
    printf("GO! Rotate now...\r\n");

    uint8_t buf[6] = {0};
    uint8_t cmd;

    float min_x =  99999.0f, max_x = -99999.0f;
    float min_y =  99999.0f, max_y = -99999.0f;
    float min_z =  99999.0f, max_z = -99999.0f;

    uint32_t start      = HAL_GetTick();
    uint32_t last_print = 0;

    while (HAL_GetTick() - start < 30000)
    {
        cmd = 0x01;
        HAL_I2C_Mem_Write(dev->hi2c, MAG_ADDR, MAG_REG_CTRL, 1, &cmd, 1, 100);
        HAL_Delay(10);
        HAL_I2C_Mem_Read(dev->hi2c, MAG_ADDR, MAG_REG_DATA, 1, buf, 6, 100);

        int16_t xr = (int16_t)(buf[0] | (buf[1] << 8));
        int16_t yr = (int16_t)(buf[2] | (buf[3] << 8));
        int8_t  zr = (int8_t)buf[4];

        float xs = xr * MAG_SENSITIVITY;
        float ys = yr * MAG_SENSITIVITY;
        float zs = zr * MAG_SENSITIVITY;

        if (xs < min_x) min_x = xs;
        if (xs > max_x) max_x = xs;
        if (ys < min_y) min_y = ys;
        if (ys > max_y) max_y = ys;
        if (zs < min_z) min_z = zs;
        if (zs > max_z) max_z = zs;

        if (HAL_GetTick() - last_print >= 1000)
        {
            uint32_t elapsed = (HAL_GetTick() - start) / 1000;
            printf("%lu s  X[%.1f, %.1f]  Y[%.1f, %.1f]  Z[%.1f, %.1f]\r\n",
                   elapsed, min_x, max_x, min_y, max_y, min_z, max_z);
            last_print = HAL_GetTick();
        }
    }

    dev->bias_x = (max_x + min_x) / 2.0f;
    dev->bias_y = (max_y + min_y) / 2.0f;
    dev->bias_z = (max_z + min_z) / 2.0f;

    printf("Mag calibration complete.\r\n");
    printf("Bias X: %.4f  Y: %.4f  Z: %.4f uT\r\n",
           dev->bias_x, dev->bias_y, dev->bias_z);
    printf("Min  X: %.2f  Y: %.2f  Z: %.2f uT\r\n", min_x, min_y, min_z);
    printf("Max  X: %.2f  Y: %.2f  Z: %.2f uT\r\n", max_x, max_y, max_z);
    printf("Save these bias values to hardcode in MAG_Init later.\r\n");
}

#include "mpu6050.h"
#include <stdio.h>

/* ── LSB Sensitivity ───────────────────────────────────────── */
static float accel_lsb(MPU6050_AccelScale s)
{
    switch (s) {
        case ACCEL_SCALE_2G:  return 16384.0f;
        case ACCEL_SCALE_4G:  return  8192.0f;
        case ACCEL_SCALE_8G:  return  4096.0f;
        case ACCEL_SCALE_16G: return  2048.0f;
        default:              return 16384.0f;
    }
}

static float gyro_lsb(MPU6050_GyroScale s)
{
    switch (s) {
        case GYRO_SCALE_250:  return 131.0f;
        case GYRO_SCALE_500:  return  65.5f;
        case GYRO_SCALE_1000: return  32.8f;
        case GYRO_SCALE_2000: return  16.4f;
        default:              return 131.0f;
    }
}

/* ── Low-level I2C ─────────────────────────────────────────── */
static MPU6050_Status write_reg(MPU6050 *dev, uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = { reg, val };
    if (HAL_I2C_Master_Transmit(dev->hi2c, MPU6050_ADDR,
                                 buf, 2, HAL_MAX_DELAY) != HAL_OK)
        return MPU6050_ERROR;
    return MPU6050_OK;
}

static MPU6050_Status read_regs(MPU6050 *dev, uint8_t reg,
                                 uint8_t *buf, uint16_t len)
{
    if (HAL_I2C_Master_Transmit(dev->hi2c, MPU6050_ADDR,
                                 &reg, 1, HAL_MAX_DELAY) != HAL_OK)
        return MPU6050_ERROR;
    if (HAL_I2C_Master_Receive(dev->hi2c, MPU6050_ADDR,
                                buf, len, HAL_MAX_DELAY) != HAL_OK)
        return MPU6050_ERROR;
    return MPU6050_OK;
}

/* ── Init ──────────────────────────────────────────────────── */
MPU6050_Status MPU6050_Init(MPU6050 *dev, I2C_HandleTypeDef *hi2c,
                             MPU6050_AccelScale ascale,
                             MPU6050_GyroScale  gscale)
{
    dev->hi2c        = hi2c;
    dev->accel_scale = ascale;
    dev->gyro_scale  = gscale;

    /* Hardcoded calibration for this unit —
       replace these values after recalibrating */
    dev->gyro_bias_x  =  0.0009f;
    dev->gyro_bias_y  =  0.2717f;
    dev->gyro_bias_z  = -1.5936f;
    dev->accel_bias_x =  -0.0114f;
    dev->accel_bias_y = -0.0145f;
    dev->accel_bias_z = -2.0545f;

    /* WHO_AM_I check */
    uint8_t who = 0;
    if (read_regs(dev, MPU6050_REG_WHO_AM_I, &who, 1) != MPU6050_OK)
        return MPU6050_ERROR;
    if (who != 0x68)
        return MPU6050_ERROR;

    /* Wake up — clear sleep bit, use gyro X as clock source */
    if (write_reg(dev, MPU6050_REG_PWR_MGMT_1, 0x01) != MPU6050_OK)
        return MPU6050_ERROR;
    HAL_Delay(100);

    /* Sample rate divider — 1 kHz output */
    if (write_reg(dev, MPU6050_REG_SMPLRT_DIV, 0x00) != MPU6050_OK)
        return MPU6050_ERROR;

    /* DLPF — 94 Hz bandwidth */
    if (write_reg(dev, MPU6050_REG_CONFIG, 0x02) != MPU6050_OK)
        return MPU6050_ERROR;

    /* Accel full-scale range */
    if (write_reg(dev, MPU6050_REG_ACCEL_CONFIG,
                  (uint8_t)ascale) != MPU6050_OK)
        return MPU6050_ERROR;

    /* Gyro full-scale range */
    if (write_reg(dev, MPU6050_REG_GYRO_CONFIG,
                  (uint8_t)gscale) != MPU6050_OK)
        return MPU6050_ERROR;

    return MPU6050_OK;
}

/* ── Read Raw ──────────────────────────────────────────────── */
MPU6050_Status MPU6050_ReadRaw(MPU6050 *dev, MPU6050_RawData *out)
{
    uint8_t buf[14];
    if (read_regs(dev, MPU6050_REG_ACCEL_XOUT_H, buf, 14) != MPU6050_OK)
        return MPU6050_ERROR;

    out->accel.x     = (int16_t)((buf[0]  << 8) | buf[1]);
    out->accel.y     = (int16_t)((buf[2]  << 8) | buf[3]);
    out->accel.z     = (int16_t)((buf[4]  << 8) | buf[5]);
    out->temperature = (int16_t)((buf[6]  << 8) | buf[7]);
    out->gyro.x      = (int16_t)((buf[8]  << 8) | buf[9]);
    out->gyro.y      = (int16_t)((buf[10] << 8) | buf[11]);
    out->gyro.z      = (int16_t)((buf[12] << 8) | buf[13]);

    return MPU6050_OK;
}

/* ── Read Scaled ───────────────────────────────────────────── */
MPU6050_Status MPU6050_ReadScaled(MPU6050 *dev, MPU6050_ScaledData *out)
{
    MPU6050_RawData raw;
    if (MPU6050_ReadRaw(dev, &raw) != MPU6050_OK)
        return MPU6050_ERROR;

    float alsb = accel_lsb(dev->accel_scale);
    float glsb = gyro_lsb(dev->gyro_scale);

    out->accel.x = ((float)raw.accel.x / alsb) - dev->accel_bias_x;
    out->accel.y = ((float)raw.accel.y / alsb) - dev->accel_bias_y;
    out->accel.z = ((float)raw.accel.z / alsb) - dev->accel_bias_z;

    out->gyro.x  = ((float)raw.gyro.x  / glsb) - dev->gyro_bias_x;
    out->gyro.y  = ((float)raw.gyro.y  / glsb) - dev->gyro_bias_y;
    out->gyro.z  = ((float)raw.gyro.z  / glsb) - dev->gyro_bias_z;

    /* Datasheet formula */
    out->temperature = ((float)raw.temperature / 340.0f) + 36.53f;

    return MPU6050_OK;
}

/* ── Gyroscope Calibration ─────────────────────────────────── */
void Gyro_Calibrate(MPU6050 *dev)
{
    printf("Gyro calibration starting in 3 seconds...\r\n");
    printf("Place sensor on flat surface and DO NOT touch it.\r\n");
    HAL_Delay(3000);
    printf("Collecting samples...\r\n");

    float sum_x = 0.0f, sum_y = 0.0f, sum_z = 0.0f;
    const int samples = 500;
    MPU6050_ScaledData imu_data;

    for (int i = 0; i < samples; i++)
    {
        MPU6050_ReadScaled(dev, &imu_data);
        sum_x += imu_data.gyro.x;
        sum_y += imu_data.gyro.y;
        sum_z += imu_data.gyro.z;
        HAL_Delay(10);
    }

    dev->gyro_bias_x = sum_x / samples;
    dev->gyro_bias_y = sum_y / samples;
    dev->gyro_bias_z = sum_z / samples;

    printf("Gyro calibration complete.\r\n");
    printf("Bias X: %.4f  Y: %.4f  Z: %.4f deg/s\r\n",
           dev->gyro_bias_x, dev->gyro_bias_y, dev->gyro_bias_z);
    printf("Save these values to hardcode in MPU6050_Init later.\r\n");
}

/* ── Accelerometer Calibration ─────────────────────────────── */
void Accel_Calibrate(MPU6050 *dev)
{
    printf("Accel calibration starting in 3 seconds...\r\n");
    printf("Place sensor FLAT and LEVEL, face UP (Z pointing to ceiling).\r\n");
    printf("DO NOT touch it during collection.\r\n");
    HAL_Delay(3000);
    printf("Collecting samples...\r\n");

    float sum_x = 0.0f, sum_y = 0.0f, sum_z = 0.0f;
    const int samples = 500;
    MPU6050_ScaledData imu_data;

    for (int i = 0; i < samples; i++)
    {
        MPU6050_ReadScaled(dev, &imu_data);
        sum_x += imu_data.accel.x;
        sum_y += imu_data.accel.y;
        sum_z += imu_data.accel.z;
        HAL_Delay(10);
    }

    dev->accel_bias_x = sum_x / samples;
    dev->accel_bias_y = sum_y / samples;
    dev->accel_bias_z = (sum_z / samples) - 1.0f;  /* remove 1g from Z */

    printf("Accel calibration complete.\r\n");
    printf("Bias X: %.4f  Y: %.4f  Z: %.4f g\r\n",
           dev->accel_bias_x, dev->accel_bias_y, dev->accel_bias_z);
    printf("Save these values to hardcode in MPU6050_Init later.\r\n");
}

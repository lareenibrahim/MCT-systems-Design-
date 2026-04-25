#include "mpu6050.h"

/* ── LSB Sensitivity lookup ────────────────────────────────── */
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

/* ── Low-level helpers ─────────────────────────────────────── */
static MPU6050_Status write_reg(MPU6050 *dev, uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = {reg, val};
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

    /* WHO_AM_I check */
    uint8_t who = 0;
    if (read_regs(dev, MPU6050_REG_WHO_AM_I, &who, 1) != MPU6050_OK)
        return MPU6050_ERROR;
    if (who != 0x68)
        return MPU6050_ERROR;

    /* Wake up — clear sleep bit */
    if (write_reg(dev, MPU6050_REG_PWR_MGMT_1, 0x00) != MPU6050_OK)
        return MPU6050_ERROR;

    HAL_Delay(100);  // let sensor stabilise

    /* Sample rate divider — 1 kHz / (1+0) = 1 kHz */
    if (write_reg(dev, MPU6050_REG_SMPLRT_DIV, 0x00) != MPU6050_OK)
        return MPU6050_ERROR;

    /* DLPF config — bandwidth ~94 Hz */
    if (write_reg(dev, MPU6050_REG_CONFIG, 0x02) != MPU6050_OK)
        return MPU6050_ERROR;

    /* Accel full-scale */
    if (write_reg(dev, MPU6050_REG_ACCEL_CONFIG,
                  (uint8_t)ascale) != MPU6050_OK)
        return MPU6050_ERROR;

    /* Gyro full-scale */
    if (write_reg(dev, MPU6050_REG_GYRO_CONFIG,
                  (uint8_t)gscale) != MPU6050_OK)
        return MPU6050_ERROR;

    return MPU6050_OK;
}

/* ── Read Raw ──────────────────────────────────────────────── */
MPU6050_Status MPU6050_ReadRaw(MPU6050 *dev, MPU6050_RawData *out)
{
    uint8_t buf[14];

    /* Read 14 bytes starting at ACCEL_XOUT_H:
       [0-5]  accel X/Y/Z
       [6-7]  temp
       [8-13] gyro  X/Y/Z                                      */
    if (read_regs(dev, MPU6050_REG_ACCEL_XOUT_H, buf, 14) != MPU6050_OK)
        return MPU6050_ERROR;

    out->accel.x    = (int16_t)(buf[0]  << 8 | buf[1]);
    out->accel.y    = (int16_t)(buf[2]  << 8 | buf[3]);
    out->accel.z    = (int16_t)(buf[4]  << 8 | buf[5]);
    out->temperature= (int16_t)(buf[6]  << 8 | buf[7]);
    out->gyro.x     = (int16_t)(buf[8]  << 8 | buf[9]);
    out->gyro.y     = (int16_t)(buf[10] << 8 | buf[11]);
    out->gyro.z     = (int16_t)(buf[12] << 8 | buf[13]);

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

    out->accel.x = raw.accel.x / alsb;
    out->accel.y = raw.accel.y / alsb;
    out->accel.z = raw.accel.z / alsb;

    out->gyro.x  = raw.gyro.x  / glsb;
    out->gyro.y  = raw.gyro.y  / glsb;
    out->gyro.z  = raw.gyro.z  / glsb;

    /* MPU6050 datasheet formula */
    out->temperature = (raw.temperature / 340.0f) + 36.53f;

    return MPU6050_OK;
}

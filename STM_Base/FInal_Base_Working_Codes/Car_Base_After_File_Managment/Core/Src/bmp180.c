#include "bmp180.h"
#include <math.h>
#include <string.h>

#define I2C_TIMEOUT   100U

/* ── Internal helpers ──────────────────────────────────────── */
static BMP180_Status _write_reg(BMP180_Handle *dev,
                                 uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = { reg, val };
    if (HAL_I2C_Master_Transmit(dev->cfg.hi2c,
                                 BMP180_ADDR,
                                 buf, 2,
                                 I2C_TIMEOUT) != HAL_OK)
        return BMP180_ERR_I2C;
    return BMP180_OK;
}

static BMP180_Status _read_regs(BMP180_Handle *dev,
                                 uint8_t reg,
                                 uint8_t *buf, uint16_t len)
{
    if (HAL_I2C_Master_Transmit(dev->cfg.hi2c,
                                 BMP180_ADDR,
                                 &reg, 1,
                                 I2C_TIMEOUT) != HAL_OK)
        return BMP180_ERR_I2C;
    if (HAL_I2C_Master_Receive(dev->cfg.hi2c,
                                BMP180_ADDR,
                                buf, len,
                                I2C_TIMEOUT) != HAL_OK)
        return BMP180_ERR_I2C;
    return BMP180_OK;
}

/* ── Init ──────────────────────────────────────────────────── */
BMP180_Status BMP180_Init(BMP180_Handle *dev, BMP180_Config *cfg)
{
    if (!dev || !cfg || !cfg->hi2c)
        return BMP180_ERR_NOT_INIT;

    memcpy(&dev->cfg, cfg, sizeof(BMP180_Config));
    dev->initialized = 0;

    /* Verify chip ID */
    uint8_t id = 0;
    if (_read_regs(dev, BMP180_REG_ID, &id, 1) != BMP180_OK)
        return BMP180_ERR_I2C;
    if (id != BMP180_ID_VAL)
        return BMP180_ERR_ID;

    /* Read 22 bytes of factory calibration from EEPROM */
    uint8_t cal[22] = {0};
    if (_read_regs(dev, BMP180_REG_CALIB, cal, 22) != BMP180_OK)
        return BMP180_ERR_I2C;

    BMP180_Calib *c = &dev->calib;
    c->AC1 = (int16_t) ((cal[0]  << 8) | cal[1]);
    c->AC2 = (int16_t) ((cal[2]  << 8) | cal[3]);
    c->AC3 = (int16_t) ((cal[4]  << 8) | cal[5]);
    c->AC4 = (uint16_t)((cal[6]  << 8) | cal[7]);
    c->AC5 = (uint16_t)((cal[8]  << 8) | cal[9]);
    c->AC6 = (uint16_t)((cal[10] << 8) | cal[11]);
    c->B1  = (int16_t) ((cal[12] << 8) | cal[13]);
    c->B2  = (int16_t) ((cal[14] << 8) | cal[15]);
    c->MB  = (int16_t) ((cal[16] << 8) | cal[17]);
    c->MC  = (int16_t) ((cal[18] << 8) | cal[19]);
    c->MD  = (int16_t) ((cal[20] << 8) | cal[21]);

    dev->initialized = 1;
    return BMP180_OK;
}

/* ── Read Scaled ───────────────────────────────────────────── */
BMP180_Status BMP180_ReadScaled(BMP180_Handle *dev, IMU_Data *out)
{
    if (!dev->initialized)
        return BMP180_ERR_NOT_INIT;

    uint8_t oss = (uint8_t)dev->cfg.oss;

    /* Step 1: Trigger and read raw temperature */
    if (_write_reg(dev, BMP180_REG_CTRL, BMP180_CMD_TEMP) != BMP180_OK)
        return BMP180_ERR_I2C;
    HAL_Delay(5);

    uint8_t buf[3] = {0};
    if (_read_regs(dev, BMP180_REG_DATA_MSB, buf, 2) != BMP180_OK)
        return BMP180_ERR_I2C;
    int32_t UT = (int32_t)((buf[0] << 8) | buf[1]);

    /* Step 2: Trigger and read raw pressure */
    uint8_t cmd = BMP180_CMD_PRESSURE_BASE + (oss << 6);
    if (_write_reg(dev, BMP180_REG_CTRL, cmd) != BMP180_OK)
        return BMP180_ERR_I2C;

    const uint8_t delays_ms[4] = { 5, 8, 14, 26 };
    HAL_Delay(delays_ms[oss]);

    if (_read_regs(dev, BMP180_REG_DATA_MSB, buf, 3) != BMP180_OK)
        return BMP180_ERR_I2C;
    int32_t UP = (int32_t)(((buf[0] << 16) | (buf[1] << 8) | buf[2])
                            >> (8 - oss));

    /* Step 3: Apply datasheet calibration formulas exactly */
    BMP180_Calib *c = &dev->calib;

    int32_t X1 = ((UT - (int32_t)c->AC6) * (int32_t)c->AC5) >> 15;
    int32_t X2 = ((int32_t)c->MC << 11) / (X1 + (int32_t)c->MD);
    int32_t B5 = X1 + X2;

    /* Temperature */
    out->baro_temperature = (float)((B5 + 8) >> 4) / 10.0f;

    /* Pressure */
    int32_t B6 = B5 - 4000;
    X1 = ((int32_t)c->B2 * ((B6 * B6) >> 12)) >> 11;
    X2 = ((int32_t)c->AC2 * B6) >> 11;
    int32_t X3 = X1 + X2;
    int32_t B3 = ((((int32_t)c->AC1 * 4 + X3) << oss) + 2) / 4;

    X1 = ((int32_t)c->AC3 * B6) >> 13;
    X2 = ((int32_t)c->B1 * ((B6 * B6) >> 12)) >> 16;
    X3 = ((X1 + X2) + 2) >> 2;
    uint32_t B4 = ((uint32_t)c->AC4 * (uint32_t)(X3 + 32768)) >> 15;
    uint32_t B7 = ((uint32_t)(UP - B3) * (50000UL >> oss));

    int32_t p;
    if (B7 < 0x80000000UL)
        p = (int32_t)((B7 * 2) / B4);
    else
        p = (int32_t)((B7 / B4) * 2);

    X1 = (p >> 8) * (p >> 8);
    X1 = (X1 * 3038) >> 16;
    X2 = (-7357 * p) >> 16;
    p  = p + ((X1 + X2 + 3791) >> 4);

    /* Pa to hPa */
    out->pressure = (float)p / 100.0f;

    /* Altitude */
    out->altitude = 44330.0f *
                    (1.0f - powf(out->pressure / dev->cfg.sea_level_hpa,
                                 0.1903f));

    return BMP180_OK;
}

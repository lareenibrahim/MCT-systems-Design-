#ifndef INC_BMP180_H_
#define INC_BMP180_H_

#include "stm32f4xx_hal.h"
#include "imu_interface.h"

/* ── I2C Address ───────────────────────────────────────────── */
#define BMP180_ADDR              (0x77 << 1)

/* ── Registers ─────────────────────────────────────────────── */
#define BMP180_REG_CALIB         0xAA
#define BMP180_REG_CTRL          0xF4
#define BMP180_REG_DATA_MSB      0xF6
#define BMP180_REG_ID            0xD0
#define BMP180_ID_VAL            0x55

/* ── Commands ──────────────────────────────────────────────── */
#define BMP180_CMD_TEMP          0x2E
#define BMP180_CMD_PRESSURE_BASE 0x34

/* ── Oversampling ──────────────────────────────────────────── */
typedef enum {
    BMP180_OSS_0 = 0,   /* ultra low power  */
    BMP180_OSS_1 = 1,   /* standard         */
    BMP180_OSS_2 = 2,   /* high res         */
    BMP180_OSS_3 = 3,   /* ultra high res   */
} BMP180_OSS;

/* ── Status ────────────────────────────────────────────────── */
typedef enum {
    BMP180_OK           = 0,
    BMP180_ERR_I2C      = 1,
    BMP180_ERR_ID       = 2,
    BMP180_ERR_NOT_INIT = 3,
} BMP180_Status;

/* ── Calibration Coefficients ──────────────────────────────── */
typedef struct {
    int16_t  AC1, AC2, AC3;
    uint16_t AC4, AC5, AC6;
    int16_t  B1,  B2;
    int16_t  MB,  MC,  MD;
} BMP180_Calib;

/* ── Config ────────────────────────────────────────────────── */
typedef struct {
    I2C_HandleTypeDef  *hi2c;
    BMP180_OSS          oss;
    float               sea_level_hpa;
} BMP180_Config;

/* ── Handle ────────────────────────────────────────────────── */
typedef struct {
    BMP180_Config  cfg;
    BMP180_Calib   calib;
    uint8_t        initialized;
} BMP180_Handle;

/* ── Default Config ────────────────────────────────────────── */
#define BMP180_DEFAULT_CONFIG(_hi2c_ptr) { \
    .hi2c          = (_hi2c_ptr),          \
    .oss           = BMP180_OSS_3,         \
    .sea_level_hpa = 1013.25f,             \
}

/* ── Public Functions ──────────────────────────────────────── */
BMP180_Status BMP180_Init      (BMP180_Handle *dev, BMP180_Config *cfg);
BMP180_Status BMP180_ReadScaled(BMP180_Handle *dev, IMU_Data *out);

#endif /* INC_BMP180_H_ */

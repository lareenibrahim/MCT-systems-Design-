#ifndef INC_MPU6050_H_
#define INC_MPU6050_H_

#include "stm32f4xx_hal.h"

/* ── I2C Address ───────────────────────────────────────────── */
#define MPU6050_ADDR            (0x68 << 1)   // AD0 = GND

/* ── Register Map ──────────────────────────────────────────── */
#define MPU6050_REG_PWR_MGMT_1  0x6B
#define MPU6050_REG_SMPLRT_DIV  0x19
#define MPU6050_REG_CONFIG      0x1A
#define MPU6050_REG_GYRO_CONFIG 0x1B
#define MPU6050_REG_ACCEL_CONFIG 0x1C
#define MPU6050_REG_ACCEL_XOUT_H 0x3B
#define MPU6050_REG_TEMP_OUT_H  0x41
#define MPU6050_REG_GYRO_XOUT_H 0x43
#define MPU6050_REG_WHO_AM_I    0x75

/* ── Scale Enums ───────────────────────────────────────────── */
typedef enum {
    ACCEL_SCALE_2G  = 0x00,
    ACCEL_SCALE_4G  = 0x08,
    ACCEL_SCALE_8G  = 0x10,
    ACCEL_SCALE_16G = 0x18
} MPU6050_AccelScale;

typedef enum {
    GYRO_SCALE_250  = 0x00,
    GYRO_SCALE_500  = 0x08,
    GYRO_SCALE_1000 = 0x10,
    GYRO_SCALE_2000 = 0x18
} MPU6050_GyroScale;

/* ── Status ────────────────────────────────────────────────── */
typedef enum {
    MPU6050_OK    = 0,
    MPU6050_ERROR = 1
} MPU6050_Status;

/* ── Raw Data ──────────────────────────────────────────────── */
typedef struct {
    int16_t x, y, z;
} MPU6050_RawVec;

typedef struct {
    MPU6050_RawVec accel;
    MPU6050_RawVec gyro;
    int16_t        temperature;
} MPU6050_RawData;

/* ── Scaled Data ───────────────────────────────────────────── */
typedef struct {
    float x, y, z;
} MPU6050_Vec;

typedef struct {
    MPU6050_Vec accel;        // in g
    MPU6050_Vec gyro;         // in deg/s
    float       temperature;  // in Celsius
} MPU6050_ScaledData;

/* ── Device Handle ─────────────────────────────────────────── */
typedef struct {
    I2C_HandleTypeDef *hi2c;
    MPU6050_AccelScale accel_scale;
    MPU6050_GyroScale  gyro_scale;
} MPU6050;

/* ── Function Prototypes ───────────────────────────────────── */
MPU6050_Status MPU6050_Init(MPU6050 *dev, I2C_HandleTypeDef *hi2c,
                             MPU6050_AccelScale ascale,
                             MPU6050_GyroScale  gscale);

MPU6050_Status MPU6050_ReadRaw   (MPU6050 *dev, MPU6050_RawData    *out);
MPU6050_Status MPU6050_ReadScaled(MPU6050 *dev, MPU6050_ScaledData *out);

#endif /* INC_MPU6050_H_ */

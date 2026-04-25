#ifndef INC_HIWONDER_MOTOR_H_
#define INC_HIWONDER_MOTOR_H_

#include "main.h"
#include <math.h>

/* I2C Device Address (7-bit 0x34, shifted for HAL) */
#define HW_MOTOR_ADDR           (0x34 << 1)

/* Motor / Wheel Constants (GA25-370 + 97mm Mecanum) */
#define WHEEL_DIAMETER_M        0.097f
#define WHEEL_CIRCUMFERENCE     (WHEEL_DIAMETER_M * 3.14159265f)
#define COUNTS_PER_REV          2059.2f   // 11 pulses * 4 (quad) * 46.8 (gear)

/* Register Map */
#define REG_BATTERY_VOLTAGE     0x00
#define REG_MOTOR_TYPE          0x14
#define REG_MOTOR_POLARITY      0x15
#define REG_MOTOR_PWM           0x1F
#define REG_MOTOR_SPEED         0x33
#define REG_MOTOR_ENCODER_TOTAL 0x3C

/* Type aliases */
typedef uint8_t  u8;
typedef int8_t   s8;
typedef uint16_t u16;
typedef int16_t  s16;
typedef int32_t  s32;

/* ── Init: call once, passing whichever I2C handle you use ── */
void              Motor_Init(I2C_HandleTypeDef *hi2c);

/* ── Configuration ── */
HAL_StatusTypeDef Motor_SetType(u8 type);
HAL_StatusTypeDef Motor_SetPolarity(u8 polarity);
HAL_StatusTypeDef Motor_ResetEncoders(void);

/* ── Control & Telemetry ── */
HAL_StatusTypeDef Motor_SetSpeeds(s16 m1, s16 m2, s16 m3, s16 m4);
HAL_StatusTypeDef Motor_ReadEncoders(s32 *enc_array);
HAL_StatusTypeDef Motor_ReadVoltage(u16 *voltage_mv);
HAL_StatusTypeDef Motor_GetKinematics(float *distance_array, float *rpm_array);

#endif /* INC_HIWONDER_MOTOR_H_ */

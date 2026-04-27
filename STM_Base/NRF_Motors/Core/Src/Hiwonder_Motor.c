#include "Hiwonder_Motor.h"

/*
 * CHANGE 1: Removed the 'static' keyword from _hi2c.
 * Previously: static I2C_HandleTypeDef *_hi2c = NULL;
 * Now:               I2C_HandleTypeDef *_hi2c = NULL;
 *
 * Why: 'static' means the variable is private to this .c file and nothing
 * outside can see it. robot_control.c needs to use _hi2c to send PWM commands.
 * Removing static and declaring it extern in the header makes it accessible
 * from any file that includes Hiwonder_Motor.h.
 */
I2C_HandleTypeDef *_hi2c = NULL;

/* ── Private: speed clamping helper ── */
static inline s8 clamp_speed(s16 val) {
    if (val >  100) return  100;
    if (val < -100) return -100;
    return (s8)val;
}

/**
 * @brief  Store the I2C handle to use for all subsequent calls.
 *         Call this once in main() before any other Motor_ function.
 */
void Motor_Init(I2C_HandleTypeDef *hi2c) {
    _hi2c = hi2c;
}

/**
 * @brief  Set motor type. Sends one byte to REG_MOTOR_TYPE.
 */
HAL_StatusTypeDef Motor_SetType(u8 type) {
    return HAL_I2C_Mem_Write(_hi2c, HW_MOTOR_ADDR, REG_MOTOR_TYPE, 1, &type, 1, 100);
}

/**
 * @brief  Set encoder polarity. Keep at 0.
 */
HAL_StatusTypeDef Motor_SetPolarity(u8 polarity) {
    return HAL_I2C_Mem_Write(_hi2c, HW_MOTOR_ADDR, REG_MOTOR_POLARITY, 1, &polarity, 1, 100);
}

/**
 * @brief  Reset all 4 encoder counters to zero.
 */
HAL_StatusTypeDef Motor_ResetEncoders(void) {
    u8 zeros[16] = {0};
    return HAL_I2C_Mem_Write(_hi2c, HW_MOTOR_ADDR, REG_MOTOR_ENCODER_TOTAL, 1, zeros, 16, 100);
}

/**
 * @brief  Set closed-loop speed for all 4 motors (-100 to 100).
 *         Uses the Hiwonder board's internal PID via REG_MOTOR_SPEED.
 *         Not used in our custom PID flow — kept for reference/testing.
 */
HAL_StatusTypeDef Motor_SetSpeeds(s16 m1, s16 m2, s16 m3, s16 m4) {
    s8 data[4] = {
        clamp_speed(m1),
        clamp_speed(m2),
        clamp_speed(m3),
        clamp_speed(m4)
    };
    return HAL_I2C_Mem_Write(_hi2c, HW_MOTOR_ADDR, REG_MOTOR_SPEED, 1, (u8*)data, 4, 100);
}

/*
 * CHANGE 2: Added Motor_SetPWM function.
 *
 * Why: Previously main.c called HAL_I2C_Mem_Write directly every time it
 * wanted to send PWM values. That raw I2C call was scattered in 3 places
 * (forward, backward, stop commands). robot_control.c also needs to send PWM
 * as the final output of the PID loop.
 *
 * This function wraps the raw I2C write into one clean call. Now anywhere that
 * needs to drive the motors just calls Motor_SetPWM(m1, m2, m3, m4) and the
 * I2C detail is hidden inside here.
 *
 * Values are clamped to -100/+100 matching the board's valid PWM range.
 */
HAL_StatusTypeDef Motor_SetPWM(s8 m1, s8 m2, s8 m3, s8 m4) {
    s8 data[4] = {m1, m2, m3, m4};
    return HAL_I2C_Mem_Write(_hi2c, HW_MOTOR_ADDR, REG_MOTOR_PWM, 1, (u8*)data, 4, 100);
}

/**
 * @brief  Read cumulative encoder pulse counts for all 4 motors.
 */
HAL_StatusTypeDef Motor_ReadEncoders(s32 *enc_array) {
    u8 raw[16];
    HAL_StatusTypeDef status = HAL_I2C_Mem_Read(_hi2c, HW_MOTOR_ADDR,
                                                  REG_MOTOR_ENCODER_TOTAL,
                                                  1, raw, 16, 100);
    if (status == HAL_OK) {
        for (int i = 0; i < 4; i++) {
            enc_array[i] = (s32)( raw[i*4]
                                | (raw[i*4+1] << 8)
                                | (raw[i*4+2] << 16)
                                | (raw[i*4+3] << 24) );
        }
    }
    return status;
}

/**
 * @brief  Read battery voltage in millivolts.
 */
HAL_StatusTypeDef Motor_ReadVoltage(u16 *voltage_mv) {
    u8 raw[2];
    HAL_StatusTypeDef status = HAL_I2C_Mem_Read(_hi2c, HW_MOTOR_ADDR,
                                                  REG_BATTERY_VOLTAGE,
                                                  1, raw, 2, 100);
    if (status == HAL_OK) {
        *voltage_mv = (u16)(raw[0] | (raw[1] << 8));
    }
    return status;
}

/**
 * @brief  Compute cumulative distance (m) and instantaneous RPM from encoders.
 *         Call this on a fixed interval for consistent RPM readings.
 */
HAL_StatusTypeDef Motor_GetKinematics(float *distance_array, float *rpm_array) {
    static s32      prev_enc[4]  = {0, 0, 0, 0};
    static uint32_t prev_tick    = 0;

    s32 enc[4];
    HAL_StatusTypeDef status = Motor_ReadEncoders(enc);

    if (status == HAL_OK) {
        uint32_t now   = HAL_GetTick();
        uint32_t dt_ms = now - prev_tick;

        for (int i = 0; i < 4; i++) {
            distance_array[i] = ((float)enc[i] / COUNTS_PER_REV) * WHEEL_CIRCUMFERENCE;

            if (prev_tick != 0 && dt_ms > 0) {
                s32   delta  = enc[i] - prev_enc[i];
                float dt_min = (float)dt_ms / 60000.0f;
                rpm_array[i] = ((float)delta / COUNTS_PER_REV) / dt_min;
            } else {
                rpm_array[i] = 0.0f;
            }
            prev_enc[i] = enc[i];
        }
        prev_tick = now;
    }
    return status;
}

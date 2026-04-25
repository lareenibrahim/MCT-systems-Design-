/**
 ******************************************************************************
 * @file    servo.c
 * @brief   Servo motor driver implementation for STM32F103C8T6 (HAL-based)
 *
 * Design decisions
 * ────────────────
 * • Timer-agnostic  – any HAL PWM-capable timer/channel can be used.
 * • No re-init      – the timer is assumed already configured by CubeMX.
 * • µs-resolution   – requires PSC=71, ARR=19999 in CubeMX (see servo.h).
 * • Angle→pulse     – linear mapping using integer arithmetic (no floats).
 ******************************************************************************
 */

#include "servo.h"

/* ═══════════════════════════ Private helpers ══════════════════════════════ */

/**
 * @brief  Map an angle [min_angle … max_angle] to a pulse width [min_pulse … max_pulse].
 *         Uses integer arithmetic to avoid pulling in the FPU / libm.
 *
 * Formula:
 *   pulse = min_pulse + (angle - min_angle) * (max_pulse - min_pulse)
 *                       ─────────────────────────────────────────────
 *                              (max_angle - min_angle)
 */
static uint16_t Servo_AngleToPulse(const Servo_Handle_t *hservo, uint8_t angle)
{
    uint32_t pulse_range = (uint32_t)(hservo->max_pulse_us - hservo->min_pulse_us);
    uint32_t angle_range = (uint32_t)(hservo->max_angle   - hservo->min_angle);
    uint32_t angle_offset = (uint32_t)(angle - hservo->min_angle);

    /* Multiply first to preserve precision, then divide */
    uint16_t pulse = hservo->min_pulse_us
                     + (uint16_t)((angle_offset * pulse_range) / angle_range);
    return pulse;
}

/**
 * @brief  Write a compare value to the timer channel.
 *         With PSC=71, ARR=19999 the CCR value == pulse width in µs.
 */
static void Servo_WriteCompare(Servo_Handle_t *hservo, uint16_t pulse_us)
{
    __HAL_TIM_SET_COMPARE(hservo->htim, hservo->channel, (uint32_t)pulse_us);
}

/* ═══════════════════════════ Public API ═══════════════════════════════════ */

/**
 * @brief  Initialise a servo instance and start the PWM output.
 */
Servo_Status_t Servo_Init(Servo_Handle_t *hservo)
{
    /* ── Validate pointer ── */
    if (hservo == NULL || hservo->htim == NULL)
    {
        return SERVO_PARAM;
    }

    /* ── Validate channel ── */
    if ((hservo->channel != TIM_CHANNEL_1) &&
        (hservo->channel != TIM_CHANNEL_2) &&
        (hservo->channel != TIM_CHANNEL_3) &&
        (hservo->channel != TIM_CHANNEL_4))
    {
        return SERVO_PARAM;
    }

    /* ── Apply defaults if user left limits at zero ── */
    if (hservo->min_pulse_us == 0U && hservo->max_pulse_us == 0U)
    {
        hservo->min_pulse_us = SERVO_DEFAULT_MIN_PULSE_US;
        hservo->max_pulse_us = SERVO_DEFAULT_MAX_PULSE_US;
    }

    if (hservo->min_angle == 0U && hservo->max_angle == 0U)
    {
        hservo->min_angle = SERVO_DEFAULT_MIN_ANGLE;
        hservo->max_angle = SERVO_DEFAULT_MAX_ANGLE;
    }

    /* ── Sanity-check limits ── */
    if (hservo->min_pulse_us >= hservo->max_pulse_us ||
        hservo->min_angle    >= hservo->max_angle)
    {
        return SERVO_PARAM;
    }

    /* ── Start PWM (timer already initialised by CubeMX) ── */
    if (HAL_TIM_PWM_Start(hservo->htim, hservo->channel) != HAL_OK)
    {
        return SERVO_ERROR;
    }

    /* ── Move to centre position ── */
    hservo->is_initialized = 1U;
    return Servo_SetCenter(hservo);
}

/* ─────────────────────────────────────────────────────────────────────────── */

/**
 * @brief  Move the servo to a specific angle.
 */
Servo_Status_t Servo_SetAngle(Servo_Handle_t *hservo, uint8_t angle)
{
    if (hservo == NULL || hservo->is_initialized == 0U)
    {
        return SERVO_PARAM;
    }

    if (angle < hservo->min_angle || angle > hservo->max_angle)
    {
        return SERVO_PARAM;
    }

    uint16_t pulse = Servo_AngleToPulse(hservo, angle);

    Servo_WriteCompare(hservo, pulse);

    hservo->current_angle    = angle;
    hservo->current_pulse_us = pulse;

    return SERVO_OK;
}

/* ─────────────────────────────────────────────────────────────────────────── */

/**
 * @brief  Set the servo position using a raw pulse width in microseconds.
 */
Servo_Status_t Servo_SetPulse(Servo_Handle_t *hservo, uint16_t pulse_us)
{
    if (hservo == NULL || hservo->is_initialized == 0U)
    {
        return SERVO_PARAM;
    }

    if (pulse_us < hservo->min_pulse_us || pulse_us > hservo->max_pulse_us)
    {
        return SERVO_PARAM;
    }

    Servo_WriteCompare(hservo, pulse_us);

    /* Back-calculate approximate angle for bookkeeping */
    uint32_t pulse_range  = (uint32_t)(hservo->max_pulse_us - hservo->min_pulse_us);
    uint32_t angle_range  = (uint32_t)(hservo->max_angle    - hservo->min_angle);
    uint32_t pulse_offset = (uint32_t)(pulse_us - hservo->min_pulse_us);

    hservo->current_angle    = hservo->min_angle
                               + (uint8_t)((pulse_offset * angle_range) / pulse_range);
    hservo->current_pulse_us = pulse_us;

    return SERVO_OK;
}

/* ─────────────────────────────────────────────────────────────────────────── */

/**
 * @brief  Move the servo to its centre position.
 */
Servo_Status_t Servo_SetCenter(Servo_Handle_t *hservo)
{
    if (hservo == NULL || hservo->is_initialized == 0U)
    {
        return SERVO_PARAM;
    }

    uint8_t center_angle = (uint8_t)((hservo->min_angle + hservo->max_angle) / 2U);
    return Servo_SetAngle(hservo, center_angle);
}

/* ─────────────────────────────────────────────────────────────────────────── */

/**
 * @brief  Read the last commanded angle.
 */
uint8_t Servo_GetAngle(const Servo_Handle_t *hservo)
{
    if (hservo == NULL || hservo->is_initialized == 0U)
    {
        return 0U;
    }
    return hservo->current_angle;
}

/* ─────────────────────────────────────────────────────────────────────────── */

/**
 * @brief  Read the last commanded pulse width.
 */
uint16_t Servo_GetPulse(const Servo_Handle_t *hservo)
{
    if (hservo == NULL || hservo->is_initialized == 0U)
    {
        return 0U;
    }
    return hservo->current_pulse_us;
}

/* ─────────────────────────────────────────────────────────────────────────── */

/**
 * @brief  Stop the PWM output.
 */
Servo_Status_t Servo_Stop(Servo_Handle_t *hservo)
{
    if (hservo == NULL || hservo->is_initialized == 0U)
    {
        return SERVO_PARAM;
    }

    if (HAL_TIM_PWM_Stop(hservo->htim, hservo->channel) != HAL_OK)
    {
        return SERVO_ERROR;
    }

    hservo->is_initialized = 0U;
    return SERVO_OK;
}

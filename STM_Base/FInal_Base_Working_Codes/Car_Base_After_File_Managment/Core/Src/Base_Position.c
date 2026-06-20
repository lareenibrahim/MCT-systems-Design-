/* Base_Position.c */
#include "Base_Position.h"
#include "sensor_sys.h"
#include <math.h>
#include <string.h>
#include <stdio.h>

extern float global_current_heading;
static float target_heading    = 0.0f;
static float KP_YAW            = 1.5f;

/* ── ROTATION CONSTANTS ── */
static const float KP_ROT = 0.2f;
static const float KI_ROT = 0.0f;
static const float KD_ROT = 8.0f;
static float rot_prev_error = 0.0f;

/* ── TRACKING VARIABLES ── */
static float    turn_target_deg      = 0.0f;
static float    turn_accumulated_deg = 0.0f;
static float    active_move_heading  = 0.0f;

static const int   ENC_SIGN[4] = { -1,  1,  1, -1 };
static const int   CMD_SIGN[4] = { -1,  1,  1, -1 };

static const float CPM[4] = {
    COUNTS_PER_REV_M1 / (3.14159f * WHEEL_DIAMETER_M),
    COUNTS_PER_REV_M2 / (3.14159f * WHEEL_DIAMETER_M),
    COUNTS_PER_REV_M3 / (3.14159f * WHEEL_DIAMETER_M),
    COUNTS_PER_REV_M4 / (3.14159f * WHEEL_DIAMETER_M),
};

static PID_t  pid[4];
static s32    enc_start[4];
static s32    target_counts[4];

volatile static int motion_active = 0;
volatile static int is_pure_turn  = 0;
static int shutdown_ticks = 0;
static int is_strafe_move = 0;
static int is_diagonal_move = 0; /* <--- ADD THIS */

/* ─────────────────────────────────────────────────────────────
 * INTERNAL HELPERS
 * ───────────────────────────────────────────────────────────── */
static float pid_compute(PID_t *p, float error)
{
    p->integral  += error;
    float deriv   = error - p->prev_error;
    p->prev_error = error;
    return KP * error + KI * p->integral + KD * deriv;
}

static int8_t clamp_cmd(float v)
{
    if (fabsf(v) < 2.5f) return 0;
    if (v >  MAX_SPEED_CMD) v =  MAX_SPEED_CMD;
    if (v < -MAX_SPEED_CMD) v = -MAX_SPEED_CMD;
    if (v > 0 && v <  MIN_SPEED_CMD) v =  MIN_SPEED_CMD;
    if (v < 0 && v > -MIN_SPEED_CMD) v = -MIN_SPEED_CMD;
    return (int8_t)v;
}

/* ─────────────────────────────────────────────────────────────
 * PUBLIC API
 * ───────────────────────────────────────────────────────────── */
void Pos_Init(void)
{
    memset(pid, 0, sizeof(pid));
    motion_active       = 0;
    is_pure_turn        = 0;
    is_strafe_move      = 0;
    shutdown_ticks      = 0;
    active_move_heading = 0.0f;
    is_diagonal_move    = 0; /* <--- ADD THIS */
}

void Pos_MoveDistance(float fwd_m, float strafe_m, float rotate_rad)
{
    active_move_heading = global_current_heading;

    target_heading = global_current_heading + (rotate_rad * 180.0f / 3.14159f);
    while (target_heading >= 360.0f) target_heading -= 360.0f;
    while (target_heading <    0.0f) target_heading += 360.0f;

    float adj_strafe = strafe_m * STRAFE_MULTIPLIER;
    float adj_rot    = rotate_rad * ROTATE_MULTIPLIER * (WHEEL_BASE_X + WHEEL_BASE_Y);

    float wd[4] = {
        ( fwd_m - adj_strafe - adj_rot),
        ( fwd_m + adj_strafe - adj_rot),
        ( fwd_m + adj_strafe + adj_rot),
        ( fwd_m - adj_strafe + adj_rot)
    };

    Motor_ReadEncoders(enc_start);
    is_pure_turn   = 0;
    shutdown_ticks = 0;
    is_strafe_move = (fabsf(strafe_m) > fabsf(fwd_m)) ? 1 : 0;

    /* --- ADD THIS NEW BLOCK --- */
        is_diagonal_move = 0;
        if (fabsf(strafe_m) > 0.01f && fabsf(fwd_m) > 0.01f) {
            is_diagonal_move = 1;
            is_strafe_move   = 0;
        }
        /* -------------------------- */

    for (int i = 0; i < 4; i++) {
        target_counts[i]    = (s32)(wd[i] * fabsf(CPM[i]));
        memset(&pid[i], 0, sizeof(PID_t));
        pid[i].prev_error   = (float)target_counts[i];
    }
    motion_active = 1;
}

void Pos_TurnAngle(float angle_degrees)
{
    active_move_heading  = global_current_heading;
    turn_target_deg      = angle_degrees;
    turn_accumulated_deg = 0.0f;

    target_heading = global_current_heading + angle_degrees;
    while (target_heading >= 360.0f) target_heading -= 360.0f;
    while (target_heading <    0.0f) target_heading += 360.0f;

    is_pure_turn   = 1;
    rot_prev_error = 0.0f;
    shutdown_ticks = 0;
    motion_active  = 1;
}

int Pos_Update(void)
{
    /* ── Already stopped — hold zero ── */
    if (!motion_active) {
        Motor_SetSpeeds(0, 0, 0, 0);
        return 1;
    }

    /* ── Shutdown coast: run a few zero-speed ticks then declare done ── */
    if (motion_active == 2) {
        Motor_SetSpeeds(0, 0, 0, 0);
        shutdown_ticks++;
        if (shutdown_ticks >= 5) {
            motion_active = 0;
            return 1;
        }
        return 0;
    }

    /* ── Normal running ── */
    s32 enc_raw[4];
    Motor_ReadEncoders(enc_raw);

    int8_t cmd[4]  = {0};
    int    terminate = 0;

    /* FIX: Use a fixed dt of 20ms (the guaranteed tick period from
     * Chassis_UpdateTick). The old approach recomputed dt here using
     * last_gyro_time, but that timestamp was already reset by
     * Chassis_UpdateTick moments before this function was called,
     * making dt ≈ 0 every tick and starving turn_accumulated_deg
     * so it never reached the target — causing infinite rotation. */
    const float dt = 0.02f;

    /* ────────────────────────────────────────────────────────
         * ROTATION PATH (Using Raw Gyro Integration)
         * ──────────────────────────────────────────────────────── */
        if (is_pure_turn)
        {
            /* FIX: POLARITY INVERSION
             * Changed += to -=. If the gyro physical mounting reads opposite
             * to the motor turning direction, we must invert it here so the
             * error shrinks instead of growing infinitely. */
            turn_accumulated_deg += (imu_data.gyro.z * dt);

            float gyro_error = turn_target_deg - turn_accumulated_deg;

            if (fabsf(gyro_error) <=10.0f) {
                terminate = 1;
            } else {
                float rot_deriv   = gyro_error - rot_prev_error;
                rot_prev_error    = gyro_error;
                float rot_cmd_out = (KP_ROT * gyro_error) + (KD_ROT * rot_deriv);

                if (rot_cmd_out >  10.0f) rot_cmd_out =  10.0f;
                if (rot_cmd_out < -10.0f) rot_cmd_out = -10.0f;
                if (rot_cmd_out > 0.0f && rot_cmd_out <  6.0f) rot_cmd_out =  6.0f;
                if (rot_cmd_out < 0.0f && rot_cmd_out > -6.0f) rot_cmd_out = -6.0f;

                for (int i = 0; i < 4; i++) {
                    float motor_corr = (i == 0 || i == 1) ? -rot_cmd_out : rot_cmd_out;
                    cmd[i] = (int8_t)(motor_corr * CMD_SIGN[i]);
                }
            }
        }
    /* ────────────────────────────────────────────────────────
     * STRAFE PATH
     * ──────────────────────────────────────────────────────── */
    else if (is_strafe_move)
    {
        float global_err_mag = 0.0f;

        for (int i = 0; i < 4; i++) {
            s32 travelled = enc_raw[i] * ENC_SIGN[i] - enc_start[i] * ENC_SIGN[i];
            float err = (float)(target_counts[i] - travelled);

            global_err_mag += fabsf(err);

            float pid_out   = pid_compute(&pid[i], err);
            float total_cmd = pid_out;

            if (fabsf(err) <= (POSITION_TOL * 3.0f)) {
                if (total_cmd >  25.0f) total_cmd =  25.0f;
                if (total_cmd < -25.0f) total_cmd = -25.0f;
            }
            // ── ANTI-STALL BOOST FOR STRAFING ──
            // Forces the motors to push through mecanum roller friction
            // Note: If 13.0f still hums, raise it to 15.0f or 18.0f!
            float strafe_min = 13.0f;
            if (total_cmd > 0.0f && total_cmd < strafe_min) total_cmd = strafe_min;
            if (total_cmd < 0.0f && total_cmd > -strafe_min) total_cmd = -strafe_min;
            cmd[i] = clamp_cmd(total_cmd * CMD_SIGN[i]);
        }

        global_err_mag /= 4.0f;

        if (global_err_mag <= (POSITION_TOL * 1.5f)) {
            terminate = 1;
        }
    }

        /* ... [YOUR UNTOUCHED STRAFE PATH ENDS HERE] ... */

            /* ────────────────────────────────────────────────────────
             * DEDICATED DIAGONAL PATH (100% ISOLATED)
             * ──────────────────────────────────────────────────────── */
            else if (is_diagonal_move)
            {
                float errors[4];
                int   all_active_done = 1;

                for (int i = 0; i < 4; i++) {
                    s32 travelled = enc_raw[i] * ENC_SIGN[i] - enc_start[i] * ENC_SIGN[i];
                    errors[i]     = (float)(target_counts[i] - travelled);

                    // In a diagonal, two wheels have targets of 0. We ignore them.
                    // We only check if the wheels that are actually supposed to move are done.
                    if (fabsf((float)target_counts[i]) > POSITION_TOL) {
                        if (fabsf(errors[i]) > POSITION_TOL) {
                            all_active_done = 0; // An active wheel is still moving
                        }
                    }
                }

                if (all_active_done) {
                    terminate = 1;
                } else {
                    for (int i = 0; i < 4; i++) {
                        // If this is a stationary wheel for this diagonal, lock it at 0
                        if (fabsf((float)target_counts[i]) <= POSITION_TOL) {
                            cmd[i] = 0;
                        }
                        else {
                            // This is an active driving wheel
                            float pid_out = pid_compute(&pid[i], errors[i]);

                            // Apply anti-stall only if it hasn't reached its target yet
                            if (fabsf(errors[i]) > POSITION_TOL) {
                                float diag_min = 10.0f; // Tune higher if motors buzz but don't move
                                if (pid_out > 0.0f && pid_out < diag_min) pid_out = diag_min;
                                if (pid_out < 0.0f && pid_out > -diag_min) pid_out = -diag_min;
                            } else {
                                // Reached target, wait for the other active wheel
                                pid_out = 0.0f;
                            }

                            cmd[i] = clamp_cmd(pid_out * CMD_SIGN[i]);
                        }
                    }
                }
            }

            /* ... [YOUR UNTOUCHED FORWARD/BACKWARD "else" BLOCK BEGINS HERE] ... */
    /* ────────────────────────────────────────────────────────
     * FORWARD / BACKWARD PATH
     *
     * Rule 1: No yaw correction — motors run at equal speed,
     *         no self-aligning at any point in the move.
     * Rule 2: As soon as the FIRST encoder reaches POSITION_TOL
     *         stop ALL motors immediately that same tick.
     * ──────────────────────────────────────────────────────── */
    else
    {
        float errors[4];
        int   any_done = 0;

        for (int i = 0; i < 4; i++) {
            s32 travelled = enc_raw[i] * ENC_SIGN[i] - enc_start[i] * ENC_SIGN[i];
            errors[i]     = (float)(target_counts[i] - travelled);

            if (fabsf(errors[i]) <= POSITION_TOL) {
                any_done = 1;
            }
        }

        if (any_done) {
            terminate = 1;
        } else {
            for (int i = 0; i < 4; i++) {
                float pid_out = pid_compute(&pid[i], errors[i]);

                // ── ANTI-STALL FLOOR ──
                // Without this, a wheel can land in clamp_cmd's 0-2.5 deadband
                // and get commanded to 0 speed even though it hasn't reached
                // POSITION_TOL yet — the wheel stalls forever, the move never
                // finishes, and the robot sits stuck on the same command.
                float fwd_min = 9.0f;
                if (pid_out > 0.0f && pid_out < fwd_min) pid_out = fwd_min;
                if (pid_out < 0.0f && pid_out > -fwd_min) pid_out = -fwd_min;

                cmd[i] = clamp_cmd(pid_out * CMD_SIGN[i]);
            }
        }
    }

    if (terminate) {
        motion_active  = 2;
        shutdown_ticks = 0;
        Motor_SetSpeeds(0, 0, 0, 0);
        return 0;
    }

    Motor_SetSpeeds(cmd[0], cmd[1], cmd[2], cmd[3]);
    return 0;
}

void Pos_Abort(void)
{
    motion_active = 0;
    Motor_SetSpeeds(0, 0, 0, 0);
}

void Pos_GetDebug(s32 *targets_out, s32 *starts_out)
{
    for (int i = 0; i < 4; i++) {
        targets_out[i] = target_counts[i];
        starts_out[i]  = enc_start[i] * ENC_SIGN[i];
    }
}

void Pos_GetErrors(int32_t *errors_out)
{
    s32 enc_raw[4];
    Motor_ReadEncoders(enc_raw);
    for (int i = 0; i < 4; i++) {
        s32 corrected_now   = enc_raw[i]   * ENC_SIGN[i];
        s32 corrected_start = enc_start[i] * ENC_SIGN[i];
        errors_out[i] = target_counts[i] - (corrected_now - corrected_start);
    }
}

// Base_Position.h
#ifndef BASE_POSITION_H
#define BASE_POSITION_H

#include "Hiwonder_Motor.h"

/* ── Tune these to your robot ── */
#define COUNTS_PER_REV_M1   1974.7f   /* replace with your actual values */
#define COUNTS_PER_REV_M2   1966.4f
#define COUNTS_PER_REV_M3   1975.0f
#define COUNTS_PER_REV_M4   1970.0f

#define WHEEL_DIAMETER_M    0.097f    /* metres */
#define WHEEL_BASE_X        0.433f    /* half track width  (m) */
#define WHEEL_BASE_Y        0.303f    /* half wheelbase    (m) */

//KP (Proportional): The main "push."
//Increase if: The robot is sluggish or stops way before reaching the target.
//Decrease if: The robot aggressively overshoots the target and has to reverse.

//KD (Derivative): The "brakes."
//Increase if: The robot is moving well but overshooting slightly at the end. KD adds damping as the error gets smaller.

//KI (Integral): The "nudge."
//Tune last: Keep this very small or 0.0. Only increase it if the robot stops just a few encoder ticks short of the target and sits there whining because KP isn't strong enough to overcome friction at that low speed.

//POSITION_TOL: Your "close enough" threshold in encoder counts.
//Increase if: The robot gets stuck in a buzzing loop at the very end.
//Decrease if: The robot is stopping too early and sacrificing accuracy.

//STRAFE_MULTIPLIER:
//If you command the robot to strafe exactly 1.0 meters to the right, but it only goes 0.8 meters, increase this multiplier (e.g., set to 1.25).

//ROTATE_MULTIPLIER:
//If you command the robot to turn 90 degrees, but it only turns 80 degrees, increase this multiplier to compensate for roller slip during rotation.

/* ── Mecanum Efficiency Multipliers ── */ // Can be changed according to the surface
#define STRAFE_MULTIPLIER   1.11f     /* Compensates for roller slip sideways */
#define ROTATE_MULTIPLIER   1.0f     /* Rotation usually needs a small bump too */

/* ── PID gains — tune these ── */
#define KP   0.05f
#define KI   0.0f
#define KD   0.4f

/* ── Speed limits ── */
#define MAX_SPEED_CMD   90    /* never exceed this — keep below DC_SPEED=80 */
#define MIN_SPEED_CMD    18    /* deadband — below this the motor won't move  */
#define POSITION_TOL    50    /* counts — "close enough" to stop             */


#define MAX_TURN_SPEED        12.0f   // max rotation command (your existing cap)
#define MIN_TURN_SPEED        12.0f   // minimum command to overcome static friction
#define TURN_BRAKE_THRESHOLD  15.0f   // degrees: enter braking state
#define TURN_BRAKE_TICKS       3      // ticks of active braking (3 × 20ms = 60ms)
#define TURN_DONE_THRESHOLD    5.0f   // degrees: acceptable final error
#define TURN_SETTLE_REQUIRED 10

typedef struct {
    float target;
    float integral;
    float prev_error;
} PID_t;

void Pos_Init(void);
void Pos_MoveDistance(float fwd_m, float strafe_m, float rotate_rad);
int  Pos_Update(void);
void Pos_Abort(void);
void Pos_GetDebug(s32 *targets_out, s32 *starts_out);
void Pos_TurnAbsolute(float degrees_change);
void Pos_GetErrors(int32_t *errors_out);
#endif

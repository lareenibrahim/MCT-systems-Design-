#ifndef ROBOT_CONTROL_H
#define ROBOT_CONTROL_H

#include "main.h"

/* ================================================================
 * ROBOT PHYSICAL DIMENSIONS
 * Measure these on your actual robot with a ruler and fill them in.
 * Lx = distance in meters from robot center to front/rear axle centerline
 * Ly = distance in meters from robot center to left/right wheel centerline
 * ================================================================ */
#define WHEEL_RADIUS_M      0.0485f   // 97mm diameter / 2
#define LX                  0.157f     // MEASURE AND REPLACE — half wheelbase (m)
#define LY                  0.205f     // MEASURE AND REPLACE — half track width (m)

/* ================================================================
 * MANUAL CONTROL SPEEDS
 * These are the velocities sent to the robot when you press a button.
 * After your first drive test, adjust these up or down to feel right.
 * Start conservative — easier to increase than to deal with a crash.
 * ================================================================ */
#define MANUAL_VX           0.3f      // m/s  — forward / backward speed
#define MANUAL_VY           0.3f      // m/s  — strafe left / right speed
#define MANUAL_OMEGA        1.0f      // rad/s — rotation speed

/* ================================================================
 * PID GAINS
 * Replace these with the values you get from MATLAB after the step
 * response test. The values here are placeholders only and will not
 * give good performance — do not skip the MATLAB step.
 * ================================================================ */
#define PID_KP              0.8f
#define PID_KI              0.0f
#define PID_KD              0.0f

/* ================================================================
 * PID LIMITS
 * DEADZONE_PWM : fill in the minimum PWM from your calibration sweep.
 *                Below this value the motor does not move, so there is
 *                no point sending a PWM smaller than this.
 * MAX_INTEGRAL : caps the integral term to prevent windup. If the robot
 *                is stopped against a wall and the integral keeps growing,
 *                this clamp stops it from exploding when the robot moves.
 * MAX_PWM      : output clamp — never send more than 100 to the board.
 * ================================================================ */
#define DEADZONE_PWM        15.0f     // REPLACE after calibration sweep
#define MAX_INTEGRAL        50.0f
#define MAX_PWM             100.0f

/* ================================================================
 * PID STRUCT
 * One instance of this exists per motor (4 total).
 * Holds all the state the PID needs between cycles.
 * You never access these fields directly from main.c —
 * the Robot_ functions manage them internally.
 * ================================================================ */
typedef struct {
    float kp, ki, kd;
    float target_rpm;    // set each cycle by inverse kinematics output
    float integral;      // accumulated error — reset on stop
    float prev_error;    // error from last cycle — used for D term
    uint32_t last_tick;  // timestamp of last update for dt calculation
} MotorPID;

/* ================================================================
 * CONTROLLER INPUT STRUCT
 * Fill this from your NRF receive callback in main.c.
 * Each field is 1 if that button is pressed, 0 if not.
 * You will add more fields here later when you have diagonal
 * movement or speed scaling from an analog joystick.
 * ================================================================ */
typedef struct {
    uint8_t forward;
    uint8_t backward;
    uint8_t strafe_left;
    uint8_t strafe_right;
    uint8_t rotate_left;
    uint8_t rotate_right;
    uint8_t stop;
} ControllerInput;

/* ================================================================
 * FUNCTION DECLARATIONS
 * ================================================================ */

/* Call once after Motor_Init — sets up the 4 PID structs */
void  Robot_Init(void);

/* Call every time a new controller packet arrives —
   maps buttons to velocity, runs inverse kinematics,
   stores RPM targets in PID structs */
void  Robot_UpdateControl(ControllerInput *input);

/* Call on a fixed interval (every 20ms) —
   reads actual RPMs, runs all 4 PIDs, sends PWM to motors */
void  Robot_PIDUpdate(float *actual_rpms);

/* Inverse kinematics — converts (Vx, Vy, omega) to 4 wheel RPM targets.
   You do not call this directly from main.c — Robot_UpdateControl calls it */
void  Kinematics_Inverse(float Vx, float Vy, float omega,
                          float *rpm_FL, float *rpm_FR,
                          float *rpm_RL, float *rpm_RR);

/* PID computation for one motor — called inside Robot_PIDUpdate for each motor.
   You do not call this directly from main.c */
float PID_Compute(MotorPID *pid, float actual_rpm);


void Robot_SetGains(char which, float val);

void Robot_GetTargets(float targets[4]);

#endif /* ROBOT_CONTROL_H */

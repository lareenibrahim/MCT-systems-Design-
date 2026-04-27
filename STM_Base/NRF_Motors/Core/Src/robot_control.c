#include "robot_control.h"
#include "Hiwonder_Motor.h"

/* ================================================================
 * MOTOR INDEX AND DIRECTION TABLE
 *
 * Motor index convention — edit this comment to match what you
 * found during your direction test:
 *   index 0 = RR (Rear right)
 *   index 1 = FR (Forward right)
 *   index 2 = RL (Rear left)
 *   index 3 = FL (Forward left)
 *
 * ================================================================ */
static const float motor_dir[4] = {
    -1.0f,   // index 0 RR
    +1.0f,   // index 1 FR
    +1.0f,   // index 2 RL
    -1.0f    // index 3 FL
};

/* ================================================================
 * FOUR PID INSTANCES — one per motor
 * Declared here, managed entirely inside this file.
 * main.c never touches these directly.
 * ================================================================ */
static MotorPID pid[4];

/* ================================================================
 * Robot_Init
 *
 * What it does:
 * Initializes all 4 PID structs with the gains defined in the header.
 * Zeros out all state (integral, prev_error, last_tick).
 *
 * When to call it:
 * Once in main.c after Motor_Init, before the while loop.
 * ================================================================ */
void Robot_Init(void) {
    for (int i = 0; i < 4; i++) {
        pid[i].kp         = PID_KP;
        pid[i].ki         = PID_KI;
        pid[i].kd         = PID_KD;
        pid[i].target_rpm = 0.0f;
        pid[i].integral   = 0.0f;
        pid[i].prev_error = 0.0f;
        pid[i].last_tick  = HAL_GetTick();
    }
}

/* ================================================================
 * Kinematics_Inverse
 *
 * What it does:
 * Takes the desired robot-level motion (Vx, Vy, omega) and computes
 * what RPM each individual wheel needs to spin at to produce that motion.
 *
 * Vx    : forward velocity in m/s.  Positive = forward.
 * Vy    : sideways velocity in m/s. Positive = right strafe.
 * omega : rotation in rad/s.        Positive = clockwise when viewed from above.
 *
 * The mecanum wheel equations distribute the motion correctly across
 * all 4 wheels. For example, strafing right requires FL and RR to spin
 * forward while FR and RL spin backward — this function handles that
 * automatically so you never have to think about it.
 *
 * to_rpm converts the linear wheel surface speed (m/s) into RPM.
 * Formula: RPM = (linear_speed / circumference) * 60
 *          circumference = 2 * pi * radius
 *          combined: RPM = linear_speed * 60 / (2 * pi * R)
 *
 * When to call it:
 * Only called internally by Robot_UpdateControl. Not called from main.c.
 * ================================================================ */
void Kinematics_Inverse(float Vx, float Vy, float omega,
                         float *rpm_FL, float *rpm_FR,
                         float *rpm_RL, float *rpm_RR)
{
    float L      = LX + LY;
    float to_rpm = 60.0f / (2.0f * 3.14159265f * WHEEL_RADIUS_M);

    *rpm_RR = ( Vx - Vy - L * omega) * to_rpm;
    *rpm_FR = ( Vx + Vy + L * omega) * to_rpm;
    *rpm_RL = ( Vx + Vy - L * omega) * to_rpm;
    *rpm_FL = ( Vx - Vy + L * omega) * to_rpm;
}

/* ================================================================
 * PID_Compute
 *
 * What it does:
 * Runs one PID iteration for one motor and returns the PWM value
 * to send to that motor.
 *
 * Step by step inside this function:
 * 1. Compute dt — time since last call in seconds. This makes the
 *    integral and derivative terms time-correct regardless of how
 *    fast the loop runs.
 * 2. Compute error = target_rpm - actual_rpm.
 * 3. P term = Kp * error. Reacts instantly to current error.
 *    Larger error → larger correction.
 * 4. Integrate error over time. Clamp with MAX_INTEGRAL to prevent
 *    windup (integral growing huge when motor is blocked).
 *    I term = Ki * integral. Eliminates steady-state error.
 * 5. D term = Kd * (error change / dt). Reacts to how fast error
 *    is changing. Dampens overshoot when RPM is approaching target.
 * 6. Sum P + I + D to get raw PWM output.
 * 7. Apply deadzone — if output is below DEADZONE_PWM the motor
 *    won't move anyway, so snap it up to the deadzone threshold.
 *    This prevents the integral from winding up at low outputs.
 * 8. Clamp output to -100/+100.
 *
 * When to call it:
 * Only called internally by Robot_PIDUpdate. Not called from main.c.
 * ================================================================ */
float PID_Compute(MotorPID *pid, float actual_rpm) {
    uint32_t now = HAL_GetTick();
    float    dt  = (now - pid->last_tick) / 1000.0f;
    pid->last_tick = now;

    // guard against first call or stalled loop
    if (dt <= 0.0f || dt > 1.0f) return 0.0f;

    float error = pid->target_rpm - actual_rpm;

    // P term
    float P = pid->kp * error;

    // I term with anti-windup clamp
    pid->integral += error * dt;
    if      (pid->integral >  MAX_INTEGRAL) pid->integral =  MAX_INTEGRAL;
    else if (pid->integral < -MAX_INTEGRAL) pid->integral = -MAX_INTEGRAL;
    float I = pid->ki * pid->integral;

    // D term
    float derivative = (error - pid->prev_error) / dt;
    float D = pid->kd * derivative;
    pid->prev_error = error;

    float output = P + I + D;

    // deadzone: snap small outputs up to the minimum that moves the motor
    if (output > 0.0f && output <  DEADZONE_PWM) output =  DEADZONE_PWM;
    if (output < 0.0f && output > -DEADZONE_PWM) output = -DEADZONE_PWM;

    // clamp to board's valid PWM range
    if      (output >  MAX_PWM) output =  MAX_PWM;
    else if (output < -MAX_PWM) output = -MAX_PWM;

    return output;
}

/* ================================================================
 * Robot_UpdateControl
 *
 * What it does:
 * This is the function you call every time a new controller packet
 * arrives. It does three things in sequence:
 *
 * 1. Maps the button states in the ControllerInput struct to a
 *    (Vx, Vy, omega) velocity command. Each button maps to a fixed
 *    speed defined in the header constants. If stop is pressed,
 *    all velocities are zeroed and the integrals are reset so there
 *    is no carryover when you start moving again.
 *
 * 2. Passes (Vx, Vy, omega) into Kinematics_Inverse to get 4
 *    individual wheel RPM targets.
 *
 * 3. Stores those targets in the pid[] structs with direction
 *    correction applied. motor_dir[] accounts for any motors whose
 *    positive direction is physically inverted.
 *
 * When to call it:
 * In main.c inside the while loop, every time an NRF packet arrives.
 * ================================================================ */
void Robot_UpdateControl(ControllerInput *input) {
    float Vx = 0.0f, Vy = 0.0f, omega = 0.0f;

    if (input->stop) {
        // zero everything and clear integrals
        for (int i = 0; i < 4; i++) {
            pid[i].target_rpm = 0.0f;
            pid[i].integral   = 0.0f;
        }
        Motor_SetPWM(0, 0, 0, 0);
        return;
    }

    // forward/backward — sets Vx
    if      (input->forward)  Vx =  MANUAL_VX;
    else if (input->backward) Vx = -MANUAL_VX;

    // strafe — sets Vy
    if      (input->strafe_right) Vy =  MANUAL_VY;
    else if (input->strafe_left)  Vy = -MANUAL_VY;

    // rotation — sets omega
    if      (input->rotate_right) omega =  MANUAL_OMEGA;
    else if (input->rotate_left)  omega = -MANUAL_OMEGA;

    // run inverse kinematics
    float rpm_FL, rpm_FR, rpm_RL, rpm_RR;
    Kinematics_Inverse(Vx, Vy, omega, &rpm_FL, &rpm_FR, &rpm_RL, &rpm_RR);

    // store targets with direction correction
    pid[0].target_rpm = rpm_RR * motor_dir[0];
    pid[1].target_rpm = rpm_FR * motor_dir[1];
    pid[2].target_rpm = rpm_RL * motor_dir[2];
    pid[3].target_rpm = rpm_FL * motor_dir[3];
}

/* ================================================================
 * Robot_PIDUpdate
 *
 * What it does:
 * This is the closed-loop control function. It runs every 20ms
 * (from the while loop delay in main.c) and does the following:
 *
 * 1. Takes the actual RPM array read from encoders (passed in from
 *    main.c after calling Motor_GetKinematics).
 * 2. Applies direction correction to the actual RPMs so the PID
 *    always sees positive = forward regardless of wiring.
 * 3. Calls PID_Compute for each motor. This compares target RPM
 *    (set by Robot_UpdateControl) against actual RPM and outputs
 *    a corrective PWM value.
 * 4. Applies direction correction to the PWM output so the physical
 *    motor spins the right way.
 * 5. Sends all 4 PWM values to the Hiwonder board in one I2C call.
 *
 * When to call it:
 * In main.c inside the while loop, every iteration, after calling
 * Motor_GetKinematics to get fresh RPM values.
 * ================================================================ */
void Robot_PIDUpdate(float *actual_rpms) {
    s8 pwm_out[4];

    for (int i = 0; i < 4; i++) {
        // apply direction correction before comparing in PID
        float corrected_rpm = actual_rpms[i] * motor_dir[i];

        // run PID — compares pid[i].target_rpm vs corrected_rpm
        float pwm = PID_Compute(&pid[i], corrected_rpm);

        // apply direction correction to output
        pwm_out[i] = (s8)(pwm * motor_dir[i]);
    }

    Motor_SetPWM(pwm_out[0], pwm_out[1], pwm_out[2], pwm_out[3]);
}

void Robot_SetGains(char which, float val) {
    for (int i = 0; i < 4; i++) {
        if (which == 'P') pid[i].kp = val;
        if (which == 'I') pid[i].ki = val;
        if (which == 'D') pid[i].kd = val;
    }
}
void Robot_GetTargets(float targets[4]) {
    for (int i = 0; i < 4; i++) {
        targets[i] = pid[i].target_rpm;
    }
}

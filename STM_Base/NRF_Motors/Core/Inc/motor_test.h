#ifndef MOTOR_TEST_H
#define MOTOR_TEST_H

#include "main.h"
#include "Hiwonder_Motor.h"

/*
 * motor_test.h / motor_test.c
 *
 * Purpose:
 * Standalone test module for motor characterization and step response logging.
 * Used ONLY for collecting data to feed into MATLAB for system identification
 * and PID gain calculation.
 *
 * Dependencies:
 * - Hiwonder_Motor.h / Hiwonder_Motor.c  (encoder reading, PWM output)
 * - Does NOT require robot_control.h or robot_control.c
 * - Does NOT use PID or kinematics — raw open loop PWM only
 *
 * How to use:
 * 1. In main.c, after Motor_Init(), call Motor_Test_Run()
 * 2. Open your serial terminal or run log_step.py on your laptop
 * 3. Follow the printed menu — type commands over USB
 * 4. When done, comment out Motor_Test_Run() in main.c to return
 *    to normal operation
 *
 * Commands available over USB serial:
 *   t  — step response test (all 4 motors, logs CSV for MATLAB)
 *   c  — PWM sweep calibration (finds deadzone, maps PWM to RPM)
 *   1  — run motor index 0 only at 30% PWM (direction/sign check)
 *   2  — run motor index 1 only at 30% PWM
 *   3  — run motor index 2 only at 30% PWM
 *   4  — run motor index 3 only at 30% PWM
 *   s  — stop all motors
 *   x  — exit test mode and return to caller
 */

/*
 * Motor_Test_Run
 * Blocking function — enters a command loop and does not return
 * until the user sends 'x'. Call this from main.c during testing,
 * comment it out for normal operation.
 */
void Motor_Test_Run(void);
void Motor_StepSingleShot(uint32_t wait_ms);
void Motor_RunContinuous(uint8_t pwm_percent);
#endif /* MOTOR_TEST_H */

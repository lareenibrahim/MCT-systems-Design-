#include "motor_test.h"
#include "usbd_cdc_if.h"
#include <string.h>
#include <stdio.h>

/* ── Private: send string over USB ── */
static void test_print(const char *msg) {
    CDC_Transmit_FS((uint8_t*)msg, strlen(msg));
    HAL_Delay(10);
}

/* ================================================================
 * flush_rx_buffer
 *
 * What it does:
 * After any test completes, leftover bytes from the Enter keypress
 * (\r, \n) stay in UserRxBufferFS. If not cleared, the next loop
 * iteration reads those bytes as a command and either executes
 * garbage or clears the buffer before your real next command arrives.
 *
 * This function waits 200ms for all trailing bytes to arrive, then
 * wipes the first 3 bytes of the buffer. It also prints a ready
 * message so you know exactly when to type your next command.
 * ================================================================ */
static void flush_rx_buffer(void) {
    extern uint8_t UserRxBufferFS[];
    HAL_Delay(200);
    UserRxBufferFS[0] = 0;
    UserRxBufferFS[1] = 0;
    UserRxBufferFS[2] = 0;
    test_print("\r\n>> Ready — type next command:\r\n");
}

/* ================================================================
 * Individual motor test
 *
 * Runs one motor at a fixed low PWM so you can:
 *   - confirm which physical wheel spins for each array index
 *   - observe encoder sign (positive or negative in telemetry)
 *   - verify spin direction matches your expected forward convention
 *
 * PWM is kept low (30%) to avoid damage if something is wrong.
 * Watch the USB telemetry while the motor runs — check ENC column.
 *
 * IMPORTANT: Lift the robot before running this test.
 * On the ground, 30% PWM may not overcome the load and the motor
 * will not spin, giving you misleading zero readings.
 * ================================================================ */
static void run_single_motor_test(uint8_t motor_index) {
    float distances[4] = {0};
    float rpms[4]      = {0};
    s32   encoders[4]  = {0};

    /* Stop all motors first so previous state does not carry over */
    Motor_SetPWM(0, 0, 0, 0);
    HAL_Delay(300);

    char msg[80];
    snprintf(msg, sizeof(msg),
        "Running motor index %d at 30%% PWM — watch ENC and RPM\r\n",
        motor_index);
    test_print(msg);

    s8 pwm[4] = {0, 0, 0, 0};
    pwm[motor_index] = 30;
    Motor_SetPWM(pwm[0], pwm[1], pwm[2], pwm[3]);

    /* Print telemetry for 4 seconds */
    for (int i = 0; i < 80; i++) {
        Motor_ReadEncoders(encoders);
        Motor_GetKinematics(distances, rpms);

        char row[120];
        snprintf(row, sizeof(row),
            "ENC:%ld,%ld,%ld,%ld | RPM:%.1f,%.1f,%.1f,%.1f\r\n",
            encoders[0], encoders[1], encoders[2], encoders[3],
            rpms[0], rpms[1], rpms[2], rpms[3]);
        test_print(row);
        HAL_Delay(50);
    }

    Motor_SetPWM(0, 0, 0, 0);
    test_print("Motor stopped.\r\n");
}

/* ================================================================
 * PWM sweep calibration
 *
 * Steps PWM from 10% to 100% in increments of 10%.
 * At each level waits 1.5 seconds for RPM to stabilise,
 * then averages 5 RPM readings.
 *
 * What this gives you:
 *   - The deadzone: first PWM level where RPM goes nonzero
 *   - PWM-to-RPM curve for all 4 motors
 *   - Whether any motor is significantly weaker than the others
 * ================================================================ */
static void run_pwm_sweep(void) {
    float distances[4] = {0};
    float rpms[4]      = {0};

    /* Stop first — ensure clean start */
    Motor_SetPWM(0, 0, 0, 0);
    HAL_Delay(300);

    test_print("PWM_PCT,RPM0,RPM1,RPM2,RPM3\r\n");

    int pwm_levels[] = {10, 20, 30, 40, 50, 60, 70, 80, 90, 100};
    int num_levels   = 10;

    for (int i = 0; i < num_levels; i++) {
        s8 pwm = (s8)pwm_levels[i];
        Motor_SetPWM(pwm, pwm, pwm, pwm);
        HAL_Delay(1500);

        float sum[4] = {0, 0, 0, 0};
        for (int s = 0; s < 5; s++) {
            Motor_GetKinematics(distances, rpms);
            for (int m = 0; m < 4; m++) sum[m] += rpms[m];
            HAL_Delay(50);
        }

        char row[100];
        snprintf(row, sizeof(row),
            "%d,%.2f,%.2f,%.2f,%.2f\r\n",
            pwm_levels[i],
            sum[0]/5.0f, sum[1]/5.0f, sum[2]/5.0f, sum[3]/5.0f);
        test_print(row);
    }

    Motor_SetPWM(0, 0, 0, 0);
    test_print("SWEEP_DONE\r\n");
}

/* ================================================================
 * Step response test
 *
 * Steps all 4 motors to 50% PWM and logs RPM every 20ms for
 * 3 seconds. Output is clean CSV for MATLAB system identification.
 *
 * Why open loop (no PID):
 * MATLAB needs the raw plant response G(s). Running with PID active
 * would identify the closed loop, not the plant, and pidtune()
 * cannot use that correctly.
 * ================================================================ */
static void run_step_response(void) {
    float distances[4] = {0};
    float rpms[4]      = {0};

    /* Stop and reset for clean baseline */
    Motor_SetPWM(0, 0, 0, 0);
    HAL_Delay(500);
    Motor_ResetEncoders();
    HAL_Delay(100);

    test_print("TIME_MS,RPM0,RPM1,RPM2,RPM3\r\n");

    Motor_SetPWM(30, 30,30, 30);

    for (int i = 0; i < 80; i++) {
        Motor_GetKinematics(distances, rpms);

        char row[80];
        snprintf(row, sizeof(row),
            "%d,%.2f,%.2f,%.2f,%.2f\r\n",
            i * 100,
            rpms[0], rpms[1], rpms[2], rpms[3]);
        test_print(row);

        HAL_Delay(100);
    }

    Motor_SetPWM(0, 0, 0, 0);
    test_print("STEP_DONE\r\n");
}

/* ================================================================
 * Motor_Test_Run
 *
 * Main entry point called from main.c.
 * Sits in a loop waiting for USB commands.
 * flush_rx_buffer() is called after every test to discard leftover
 * \r\n bytes from the Enter keypress so the next command reads clean.
 * Returns when 'x' is received.
 * ================================================================ */
void Motor_Test_Run(void) {
    extern uint8_t UserRxBufferFS[];

    /* Clear buffer on entry — discard anything queued before we started */
    UserRxBufferFS[0] = 0;
    UserRxBufferFS[1] = 0;
    UserRxBufferFS[2] = 0;

    test_print("\r\n===== MOTOR TEST MODE =====\r\n");
    test_print("Commands:\r\n");
    test_print("  t  - step response (CSV for MATLAB)\r\n");
    test_print("  c  - PWM sweep calibration\r\n");
    test_print("  1  - run motor 0 only (FL)\r\n");
    test_print("  2  - run motor 1 only (FR)\r\n");
    test_print("  3  - run motor 2 only (RL)\r\n");
    test_print("  4  - run motor 3 only (RR)\r\n");
    test_print("  s  - stop all motors\r\n");
    test_print("  x  - exit test mode\r\n");
    test_print("===========================\r\n");
    test_print(">> Ready — type next command:\r\n");

    void Motor_StepSingleShot(uint32_t wait_ms) {
        float distances[4], rpms[4];

        Motor_GetKinematics(distances, rpms);  // reset state
        HAL_Delay(50);

        Motor_SetPWM(30, 30, 30, 30);
        HAL_Delay(wait_ms);                    // wait for motor to reach this point

        Motor_GetKinematics(distances, rpms);  // flush the long window — discard this
        HAL_Delay(50);                         // wait 50ms more at steady state
        Motor_GetKinematics(distances, rpms);  // THIS is your actual reading over 50ms
        Motor_SetPWM(0, 0, 0, 0);

        char row[80];
        snprintf(row, sizeof(row),
            "t=%lums: %.2f, %.2f, %.2f, %.2f\r\n",
            wait_ms, rpms[0], rpms[1], rpms[2], rpms[3]);
        test_print(row);
    }
    void Motor_RunContinuous(uint8_t pwm_percent) {
        char msg[60];
        snprintf(msg, sizeof(msg),
            "Motors running at %d%% PWM. Type 's' to stop.\r\n",
            pwm_percent);
        test_print(msg);

        Motor_SetPWM(pwm_percent, pwm_percent, pwm_percent, pwm_percent);

        while (1) {
            if ((char)UserRxBufferFS[0] == 's') {
                Motor_SetPWM(0, 0, 0, 0);
                test_print("Motors stopped.\r\n");
                flush_rx_buffer();
                return;
            }
            HAL_Delay(20);
        }
    }
    while (1) {
        char cmd = (char)UserRxBufferFS[0];

        if (cmd != 0) {
            UserRxBufferFS[0] = 0;

            if (cmd == 't') {
                test_print("Starting step response...\r\n");
                run_step_response();
                flush_rx_buffer();
            }
            else if (cmd == 'c') {
                test_print("Starting PWM sweep...\r\n");
                run_pwm_sweep();
                flush_rx_buffer();
            }
            else if (cmd == '1') {
                run_single_motor_test(0);
                flush_rx_buffer();
            }
            else if (cmd == '2') {
                run_single_motor_test(1);
                flush_rx_buffer();
            }
            else if (cmd == '3') {
                run_single_motor_test(2);
                flush_rx_buffer();
            }
            else if (cmd == '4') {
                run_single_motor_test(3);
                flush_rx_buffer();
            }
            else if (cmd == 's') {
                Motor_SetPWM(0, 0, 0, 0);
                test_print("All motors stopped.\r\n");
                flush_rx_buffer();
            }
            else if (cmd == 'x') {
                Motor_SetPWM(0, 0, 0, 0);
                test_print("Exiting test mode.\r\n");
                flush_rx_buffer();
                return;
            }
            else if (cmd == 'r') {
                test_print("Motors running at 30% PWM. Type 's' to stop.\r\n");
                Motor_RunContinuous(30);
                flush_rx_buffer();
            }
            else if (cmd =='y'){
            	test_print("Starting single shot rise time test...\r\n");
            	test_print("WAIT_MS,RPM0,RPM1,RPM2,RPM3\r\n");

            	Motor_StepSingleShot(200);
            	HAL_Delay(3000);

            	Motor_StepSingleShot(300);
            	HAL_Delay(3000);

            	Motor_StepSingleShot(400);
            	HAL_Delay(3000);

            	Motor_StepSingleShot(500);
            	HAL_Delay(3000);

            	Motor_StepSingleShot(700);
            	HAL_Delay(3000);

            	Motor_StepSingleShot(1000);
            	HAL_Delay(3000);

            	Motor_StepSingleShot(1500);
            	HAL_Delay(3000);

            	test_print("SHOT_DONE\r\n");
            	flush_rx_buffer();
            }
            /* ignore \r \n and other whitespace silently */
        }

        HAL_Delay(20);
    }
}

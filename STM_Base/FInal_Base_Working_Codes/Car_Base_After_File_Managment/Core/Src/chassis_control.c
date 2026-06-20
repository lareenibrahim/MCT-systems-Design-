/*
 * chassis_control.c
 *
 * Created on: Jun 10, 2026
 * Author: Osama Mohammed
 */
#include "chassis_control.h"
#include "comm_parser.h"
#include "sensor_sys.h"
#include "Hiwonder_Motor.h"
#include "Base_Manual.h"
#include "Base_Position.h"
#include <string.h>
#include <stdio.h>

int robot_mode      = 0;
int auto_is_running = 0;
uint32_t last_cmd_time = 0;
static int      pos_mode         = 0;
static uint32_t last_pos_time    = 0;
static int      square_step      = 0;
static int      square_active    = 0;
static uint8_t  active_pi_cmd    = 0;

extern volatile uint8_t new_packet_ready;
extern volatile char    valid_cmd;
extern uint8_t          packet_buffer[8];
/*
extern void Pos_MoveDistance(float fwd_m, float strafe_m, float rotate_rad);
extern void Pos_TurnAngle(float angle_degrees);

static void start_move(float fwd, float strafe, float rot_rad)
{
    Pos_MoveDistance(fwd, strafe, rot_rad);
    pos_mode      = 1;
    last_pos_time = HAL_GetTick();
    last_cmd_time = HAL_GetTick();
}
*/

extern void Pos_MoveDistance(float fwd_m, float strafe_m, float rotate_rad);
extern void Pos_TurnAngle(float angle_degrees);

static void start_move(float fwd, float strafe, float rot_rad)
{
    Pos_MoveDistance(fwd, strafe, rot_rad);
    pos_mode      = 1;
    last_pos_time = HAL_GetTick();
    last_cmd_time = HAL_GetTick();
}

static void start_turn(float angle_deg)
{
    Pos_TurnAngle(angle_deg);
    pos_mode      = 1;
    last_pos_time = HAL_GetTick();
    last_cmd_time = HAL_GetTick();
}

void Chassis_Init(I2C_HandleTypeDef *hi2c)
{
    DC_Init(hi2c);
    Pos_Init();

    uint16_t test_volt = 0;
    if (Motor_ReadVoltage(&test_volt) == HAL_OK) {
        USB_Print("  -> [OK] Motor Driver Connected (I2C1)\r\n");
    } else {
        USB_Print("  -> [WARNING] Motor Driver Write OK, Read FAILED.\r\n");
    }

    uint8_t dummy = 0;
    HAL_StatusTypeDef status = HAL_I2C_Mem_Read(hi2c, 0x34<<1, 0x00, 1, &dummy, 1, 100);
    char dbuf[60];
    snprintf(dbuf, sizeof(dbuf), "  -> I2C1 raw read: status=%d val=0x%02X\r\n", status, dummy);
    USB_Print(dbuf);

    last_cmd_time = HAL_GetTick();
}

void Chassis_UpdateTick(void)
{
    if (HAL_GetTick() - last_pos_time >= 20)
    {
        last_pos_time = HAL_GetTick();

        MPU6050_ReadScaled(&imu, &imu_data);
        MAG_Read(&mag);
        global_current_heading = HeadingFusion_Update(&fusion, imu_data.gyro.z, mag.heading, 0.02f);

        if (pos_mode)
        {
            last_cmd_time = HAL_GetTick();
            int done = Pos_Update();

            if (done)
            {
                pos_mode = 0;

                // ── THE FIX: Print telemetry FIRST, then send ACK LAST ──
                // This guarantees the ACK packet is never bundled with text
                // by the CDC driver, which would cause the Pi to misparse it.
                int32_t final_errors[4];
                Pos_GetErrors(final_errors);
                char buf[150];
                snprintf(buf, sizeof(buf), "[STOP] Final Err: M1=%ld M2=%ld M3=%ld M4=%ld\r\n",
                    final_errors[0], final_errors[1], final_errors[2], final_errors[3]);
                USB_Print(buf);   // text goes out first

                if (square_active)
                {
                    square_step++;
                    char sbuf[40];
                    snprintf(sbuf, sizeof(sbuf), "[SQ] Step %d done\r\n", square_step);
                    USB_Print(sbuf);

                    switch (square_step) {
                        case 1: start_move(0.0f,  0.5f, 0.0f); break;
                        case 2: start_move(-0.5f, 0.0f, 0.0f); break;
                        case 3: start_move(0.0f, -0.5f, 0.0f); break;
                        default: square_active = 0; USB_Print("[SQ] Square complete.\r\n"); break;
                    }
                }
                else {
                    USB_Print("[POS] Move complete\r\n");
                }

                // ACK is sent AFTER all text — never bundled with telemetry
                if (active_pi_cmd != 0) {
                    Send_ACK(active_pi_cmd, 0x00);
                    active_pi_cmd = 0;
                }
            }
        }
        else if (robot_mode == 0)
        {
            DC_UpdateRamp();
        }
    }

    /* ── SAFETY WATCHDOG ── */
    if (!pos_mode && (HAL_GetTick() - last_cmd_time > 200)) {
        DC_Stop();
    }
}

void Chassis_ProcessCommands(void)
{
    extern volatile uint8_t new_packet_ready;
    extern volatile char    valid_cmd;
    extern uint8_t          packet_buffer[8];

    if (new_packet_ready)
    {
        new_packet_ready = 0;

        char current_cmd = valid_cmd;
        int is_pi_packet = 0;
        uint8_t pi_cmd = 0, b1 = 0, b2 = 0, b3 = 0, b4 = 0;

        if (packet_buffer[0] == 0xFF && packet_buffer[1] == 0xAA)
        {
            pi_cmd = packet_buffer[2];
            b1  = packet_buffer[3]; b2  = packet_buffer[4];
            b3  = packet_buffer[5]; b4  = packet_buffer[6];

            if (pi_cmd == 0x10 || pi_cmd == 0x20) {
                is_pi_packet = 1;
            } else {
                current_cmd = pi_cmd;
            }
        }

        memset(packet_buffer, 0, 8);

        /* ── STATE 1: PI AUTONOMOUS MAP MOVEMENTS ── */
        if (is_pi_packet)
        {
            float fwd_m    = (pi_cmd == 0x10) ? (float)((int16_t)((b1 << 8) | b2)) / 100.0f : 0.0f;
            float strafe_m = (pi_cmd == 0x10) ? (float)((int16_t)((b3 << 8) | b4)) / 100.0f : 0.0f;
            float angle_deg= (pi_cmd == 0x20) ? (float)((int16_t)((b1 << 8) | b2)) : 0.0f;

            if (pi_cmd == 0x30 ||
               (pi_cmd == 0x10 && fwd_m == 0.0f && strafe_m == 0.0f) ||
               (pi_cmd == 0x20 && angle_deg == 0.0f))
            {
                pos_mode        = 0;
                auto_is_running = 0;
                active_pi_cmd   = 0;
                square_active   = 0;

                Pos_Abort();
                DC_Stop();
                HAL_Delay(5);

                int32_t stuck_errors[4];
                Pos_GetErrors(stuck_errors);
                char xbuf[150];
                snprintf(xbuf, sizeof(xbuf), "[ABORT] Pi Universal STOP: M1=%ld M2=%ld M3=%ld M4=%ld\r\n",
                    stuck_errors[0], stuck_errors[1], stuck_errors[2], stuck_errors[3]);
                USB_Print("[AUTO] Pi Packet E-STOP Executed.\r\n");
                USB_Print(xbuf);
                Send_ACK(pi_cmd, 0x00);
            }
            else if (robot_mode == 1 && auto_is_running == 1)
            {
                if (pos_mode == 1)
                {
                    USB_Print("[WARN] Ignored Map Command: Robot is currently moving.\r\n");
                    Send_ACK(pi_cmd, 0x02);
                }
                else
                {
                    if (pi_cmd == 0x10) {
                        char log[80];
                        snprintf(log, sizeof(log), "[AUTO] Translate -> F:%.2fm, S:%.2fm\r\n", fwd_m, strafe_m);
                        USB_Print(log);
                        start_move(fwd_m, strafe_m, 0.0f);
                        active_pi_cmd = pi_cmd;
                    }
                    else if (pi_cmd == 0x20) {
                        char log[80];
                        snprintf(log, sizeof(log), "[AUTO] Twist -> %.1f degrees\r\n", angle_deg);
                        USB_Print(log);
                        start_turn(angle_deg);
                        active_pi_cmd = pi_cmd;
                    }
                }
            }
            else {
                USB_Print("[WARN] Ignored Map Command: Auto Mode is OFF or Paused.\r\n");
                Send_ACK(pi_cmd, 0x01);
            }
        }

        /* ── STATE 2: ASCII COMMANDS ── */
        if (current_cmd != 0)
        {
            int cmd_known      = 1;
            char robotCmdStr[3] = "S";
            int is_manual_move = 0;

            if (current_cmd == 'M')
            {
                robot_mode      = 1;
                auto_is_running = 0;
                USB_Print("[MODE] Switched to AUTONOMOUS (PID Position Control active).\r\n");
                Send_Mode_Confirm(0x01);
            }
            else if (current_cmd == 'P')
            {
                robot_mode      = 0;
                auto_is_running = 0;
                Pos_Abort();
                DC_Stop();
                USB_Print("[MODE] Switched to MANUAL (Continuous Drive active).\r\n");
                Send_Mode_Confirm(0x00);
            }
            else if (current_cmd == 'X')
            {
                pos_mode        = 0;
                square_active   = 0;
                auto_is_running = 0;
                active_pi_cmd   = 0;

                Pos_Abort();
                DC_Stop();
                HAL_Delay(5);

                int32_t stuck_errors[4];
                Pos_GetErrors(stuck_errors);
                char xbuf[150];
                snprintf(xbuf, sizeof(xbuf), "[ABORT] Stuck Err: M1=%ld M2=%ld M3=%ld M4=%ld\r\n",
                    stuck_errors[0], stuck_errors[1], stuck_errors[2], stuck_errors[3]);
                USB_Print("[STOP] EMERGENCY STOP Executed.\r\n");
                USB_Print(xbuf);
            }
            else if (robot_mode == 1)
            {
                if (current_cmd == 'N') {
                    auto_is_running = 1;
                    USB_Print("[AUTO] Pi Map Sequence STARTED!\r\n");
                }
                else if (current_cmd == '0') {
                    Motor_ResetEncoders(); HAL_Delay(50); USB_Print("[ENC] Reset.\r\n");
                }
                else if (current_cmd == '8') {
                    int32_t errs[4];
                    Pos_GetErrors(errs);
                    char buf[180];
                    snprintf(buf, sizeof(buf), "[PID] ERR: M1=%ld M2=%ld M3=%ld M4=%ld | mode=%d\r\n",
                             errs[0], errs[1], errs[2], errs[3], pos_mode);
                    USB_Print(buf);
                }
                else if (pos_mode == 1)
                {
                    USB_Print("[WARN] Ignored Keyboard Command: Robot is currently moving.\r\n");
                }
                else
                {
                    switch (current_cmd)
                    {
                        case 'F': USB_Print("[POS] Fwd 1.0m\r\n");       start_move(1.0f,  0.0f, 0.0f); break;
                        case 'B': USB_Print("[POS] Rev 1.0m\r\n");       start_move(-1.0f, 0.0f, 0.0f); break;
                        case 'L': USB_Print("[POS] Strafe R 1.0m\r\n");  start_move(0.0f,  1.0f, 0.0f); break;
                        case 'R': USB_Print("[POS] Strafe L 1.0m\r\n");  start_move(0.0f, -1.0f, 0.0f); break;
                        case 'Q': USB_Print("[POS] CW 60deg\r\n");       start_turn(60.0f);  break;
                        case 'E': USB_Print("[POS] CCW 90deg\r\n");      start_turn(-90.0f); break;
                        case '1': USB_Print("[POS] Fwd 0.2m\r\n");       start_move(0.2f,  0.0f, 0.0f); break;
                        case '2': USB_Print("[POS] Rev 0.2m\r\n");       start_move(-0.2f, 0.0f, 0.0f); break;
                        case '3': USB_Print("[POS] Strafe R 0.2m\r\n");  start_move(0.0f,  0.2f, 0.0f); break;
                        case '4': USB_Print("[POS] Strafe L 0.2m\r\n");  start_move(0.0f, -0.2f, 0.0f); break;
                        case '5': USB_Print("[POS] CW 90 (IMU)\r\n");    start_turn(-90.0f); break;
                        case '6': USB_Print("[POS] CCW 90 (IMU)\r\n");   start_turn(90.0f);  break;
                        default: cmd_known = 0; break;
                    }
                }
            }
            else if (robot_mode == 0)
            {
                switch (current_cmd)
                {
                    case 'F': strcpy(robotCmdStr, "F");  is_manual_move = 1; break;
                    case 'B': strcpy(robotCmdStr, "B");  is_manual_move = 1; break;
                    case 'L': strcpy(robotCmdStr, "L");  is_manual_move = 1; break;
                    case 'R': strcpy(robotCmdStr, "R");  is_manual_move = 1; break;
                    case 'Q': strcpy(robotCmdStr, "CL"); is_manual_move = 1; break;
                    case 'E': strcpy(robotCmdStr, "CR"); is_manual_move = 1; break;
                    case 'G': strcpy(robotCmdStr, "FL"); is_manual_move = 1; break;
                    case 'I': strcpy(robotCmdStr, "FR"); is_manual_move = 1; break;
                    case 'H': strcpy(robotCmdStr, "BL"); is_manual_move = 1; break;
                    case 'J': strcpy(robotCmdStr, "BR"); is_manual_move = 1; break;
                    case 'S': strcpy(robotCmdStr, "S");  is_manual_move = 1; break;
                    case '0': Motor_ResetEncoders(); HAL_Delay(50); USB_Print("[ENC] Reset.\r\n"); break;
                    default:  cmd_known = 0; break;
                }
            }

            if (is_manual_move && cmd_known) {
                DC_Execute(robotCmdStr);
                last_cmd_time = HAL_GetTick();
                char usblog[40];
                snprintf(usblog, sizeof(usblog), "[USB] Executed Manual: %s\r\n", robotCmdStr);
                USB_Print(usblog);
            }
        }
    }
}

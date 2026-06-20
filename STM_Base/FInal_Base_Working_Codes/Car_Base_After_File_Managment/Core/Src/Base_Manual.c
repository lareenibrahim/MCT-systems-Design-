/*
 * Base_Manual.c
 *
 * Created on: Apr 28, 2026
 * Author: Osama Mohammed
 */

#include "Base_Manual.h"
#include <string.h>

#define S   DC_SPEED
#define T   DC_TURN_SPEED
#define D   DC_DIAG_SPEED

static float current_spd[4] = {0, 0, 0, 0};
static float target_spd[4]  = {0, 0, 0, 0};

/* ── Adjust this to change how fast it accelerates ──
   This is how much speed is added every 20ms.
   If DC_SPEED is 100, a RAMP_STEP of 5.0 means it takes
   20 loops (400 milliseconds) to hit full speed. */
#define RAMP_STEP 5.0f

void DC_Init(I2C_HandleTypeDef *hi2c)
{
    Motor_Init(hi2c);
    HAL_Delay(200);
    Motor_SetType(3);
    HAL_Delay(10);
    Motor_SetPolarity(0);
    HAL_Delay(10);
    Motor_ResetEncoders();
    HAL_Delay(10);
    Motor_SetSpeeds(0, 0, 0, 0);
}

void DC_Execute(const char *cmd)
{
    if      (strcmp(cmd, "F")  == 0) { target_spd[0]= -S; target_spd[1]=  S; target_spd[2]=  S; target_spd[3]= -S; }
    else if (strcmp(cmd, "B")  == 0) { target_spd[0]=  S; target_spd[1]= -S; target_spd[2]= -S; target_spd[3]=  S; }
    else if (strcmp(cmd, "L")  == 0) { target_spd[0]=  S; target_spd[1]=  S; target_spd[2]=  S; target_spd[3]=  S; }
    else if (strcmp(cmd, "R")  == 0) { target_spd[0]= -S; target_spd[1]= -S; target_spd[2]= -S; target_spd[3]= -S; }

    else if (strcmp(cmd, "FL") == 0) { target_spd[0]=  0; target_spd[1]=  D; target_spd[2]=  D; target_spd[3]=  0; }
    else if (strcmp(cmd, "FR") == 0) { target_spd[0]= -D; target_spd[1]=  0; target_spd[2]=  0; target_spd[3]= -D; }
    else if (strcmp(cmd, "BL") == 0) { target_spd[0]=  D; target_spd[1]=  0; target_spd[2]=  0; target_spd[3]=  D; }
    else if (strcmp(cmd, "BR") == 0) { target_spd[0]=  0; target_spd[1]= -D; target_spd[2]= -D; target_spd[3]=  0; }

    else if (strcmp(cmd, "CL") == 0) { target_spd[0]=  T; target_spd[1]= -T; target_spd[2]=  T; target_spd[3]= -T; }
    else if (strcmp(cmd, "CR") == 0) { target_spd[0]= -T; target_spd[1]=  T; target_spd[2]= -T; target_spd[3]=  T; }

    else                             { target_spd[0]=  0; target_spd[1]=  0; target_spd[2]=  0; target_spd[3]=  0; }
}

void DC_Stop(void)
{
    /* Set targets to 0. DC_UpdateRamp will bring the actual speed down gradually */
    target_spd[0] = 0; target_spd[1] = 0; target_spd[2] = 0; target_spd[3] = 0;
}

/* ═══════════════════════════════════════════════════════════════════
   DC_UpdateRamp
   Called every 20ms by main.c to bridge current speed to target speed
   ═══════════════════════════════════════════════════════════════════ */
void DC_UpdateRamp(void)
{
    for(int i = 0; i < 4; i++) {
        if (current_spd[i] < target_spd[i]) {
            current_spd[i] += RAMP_STEP;
            if (current_spd[i] > target_spd[i]) current_spd[i] = target_spd[i];
        }
        else if (current_spd[i] > target_spd[i]) {
            current_spd[i] -= RAMP_STEP;
            if (current_spd[i] < target_spd[i]) current_spd[i] = target_spd[i];
        }
    }

    Motor_SetSpeeds((int)current_spd[0], (int)current_spd[1], (int)current_spd[2], (int)current_spd[3]);
}

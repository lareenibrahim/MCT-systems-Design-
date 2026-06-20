/*
 * Base_Manual.h
 *
 *  Created on: Apr 28, 2026
 *      Author: Osama Mohammed
 */

#ifndef INC_BASE_MANUAL_H_
#define INC_BASE_MANUAL_H_

#include "Hiwonder_Motor.h"

/* ═══════════════════════════════════════════════════════
   HARD-CODED SPEED  —  change this one value to tune
   Range: 1 to 100
   ═══════════════════════════════════════════════════════ */
#define DC_SPEED        80      /* straight / strafe speed  */
#define DC_TURN_SPEED   40      /* rotation speed           */
#define DC_DIAG_SPEED   40      /* diagonal speed           */

/* ═══════════════════════════════════════════════════════
   ASCII command map
   ═══════════════════════════════════════════════════════
   "F"  — Forward
   "B"  — Backward
   "L"  — Strafe Left
   "R"  — Strafe Right
   "FL" — Forward-Left  diagonal
   "FR" — Forward-Right diagonal
   "BL" — Backward-Left  diagonal
   "BR" — Backward-Right diagonal
   "CL" — spin Counter-Left  (rotate left)
   "CR" — spin Counter-Right (rotate right)
   "S"  — Stop
   ═══════════════════════════════════════════════════════ */

/* Motor layout (top-down, robot facing up)
   M1(FL) ──┬── M3(FR)
             │
   M2(RL) ──┴── M4(RR)          */

void DC_Init(I2C_HandleTypeDef *hi2c);
void DC_Execute(const char *cmd);
void DC_Stop(void); /* INC_BASE_MANUAL_H_ */
void DC_UpdateRamp(void);  // <--- Add this line
#endif


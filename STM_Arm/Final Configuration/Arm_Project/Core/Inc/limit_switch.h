/*
 * limit_switch.h
 *
 *  Created on: Apr 24, 2026
 *      Author: Mohamed
 */

#ifndef INC_LIMIT_SWITCH_H_
#define INC_LIMIT_SWITCH_H_

#include <stdint.h>
#include "main.h"

// ══════════════════════════════════════════════════════════════
// ── WIRING CONFIGURATION  (edit here only) ───────────────────
// ══════════════════════════════════════════════════════════════
//
//  SWITCH_NO → Normally Open
//  SWITCH_NC → Normally Closed
//
//  YOUR WIRING:
//    COM  → VCC (3.3 V)
//    NC   → signal pin  (with internal PULL-DOWN as failsafe)
//
//  Idle   (switch not pressed): NC shorted to COM → pin = HIGH
//  Pressed (circuit opens)    : pull-down pulls pin LOW  → FALLING edge
//
#define SWITCH_NO   0
#define SWITCH_NC   1

#define ARM_LIMIT_TYPE        SWITCH_NC
#define GRIPPER_LIMIT_TYPE    SWITCH_NC

// ══════════════════════════════════════════════════════════════
// ── GPIO Mode & Pull — derived from wiring type ──────────────
// ══════════════════════════════════════════════════════════════
//
//  NO  switch: pin idles LOW  (pull-down),  fires on RISING  (press = HIGH)
//  NC  switch with COM→VCC:
//              pin idles HIGH (driven by VCC through NC contact),
//              fires on FALLING (press opens circuit, pull-down pulls LOW)
//
#if (ARM_LIMIT_TYPE == SWITCH_NO)
    #define ARM_LIMIT_GPIO_PULL     GPIO_PULLDOWN
    #define ARM_LIMIT_GPIO_MODE     GPIO_MODE_IT_RISING
#elif (ARM_LIMIT_TYPE == SWITCH_NC)
    #define ARM_LIMIT_GPIO_PULL     GPIO_PULLDOWN   /* failsafe: holds LOW when NC opens */
    #define ARM_LIMIT_GPIO_MODE     GPIO_MODE_IT_FALLING  /* NC opens → pin falls LOW  */
#else
    #error "ARM_LIMIT_TYPE must be SWITCH_NO or SWITCH_NC"
#endif

#if (GRIPPER_LIMIT_TYPE == SWITCH_NO)
    #define GRIPPER_LIMIT_GPIO_PULL GPIO_PULLDOWN
    #define GRIPPER_LIMIT_GPIO_MODE GPIO_MODE_IT_RISING
#elif (GRIPPER_LIMIT_TYPE == SWITCH_NC)
    #define GRIPPER_LIMIT_GPIO_PULL GPIO_PULLDOWN   /* failsafe: holds LOW when NC opens */
    #define GRIPPER_LIMIT_GPIO_MODE GPIO_MODE_IT_FALLING  /* NC opens → pin falls LOW  */
#else
    #error "GRIPPER_LIMIT_TYPE must be SWITCH_NO or SWITCH_NC"
#endif

// ══════════════════════════════════════════════════════════════
// ── Homing / Backoff Parameters ──────────────────────────────
// ══════════════════════════════════════════════════════════════

#define HOMING_BACKOFF_STEPS      400
#define HOMING_BACKOFF_DELAY_US   500

// ══════════════════════════════════════════════════════════════
// ── Debounce ─────────────────────────────────────────────────
// ══════════════════════════════════════════════════════════════

#define LIMIT_DEBOUNCE_MS         50

// ══════════════════════════════════════════════════════════════
// ── Public Flags ─────────────────────────────────────────────
// ══════════════════════════════════════════════════════════════

extern volatile uint8_t arm_limit_triggered;
extern volatile uint8_t gripper_cube_detected;

// ══════════════════════════════════════════════════════════════
// ── Function Declarations ────────────────────────────────────
// ══════════════════════════════════════════════════════════════

void LimitSwitch_Init(void);
void LimitSwitch_EXTI_Callback(uint16_t GPIO_Pin);

#endif /* INC_LIMIT_SWITCH_H_ */

#ifndef INC_STEPPER_H_
#define INC_STEPPER_H_

#include "stm32f1xx_hal.h"
#include <stdint.h>

// ── Pin Definitions ───────────────────────────────────────────
#define STEP_PIN        GPIO_PIN_0
#define STEP_PORT       GPIOA

#define DIR_PIN         GPIO_PIN_7
#define DIR_PORT        GPIOA

#define EN_PIN          GPIO_PIN_0
#define EN_PORT         GPIOB

// ── Motor Physical Constants ──────────────────────────────────
#define STEPS_PER_REV_FULL    200       // NEMA17 = 1.8° per step
#define MICROSTEP_DIV         1         // 1/8 microstepping → 1600 steps/rev

// ── Direction Constants ───────────────────────────────────────
#define STEPPER_CW            GPIO_PIN_SET
#define STEPPER_CCW           GPIO_PIN_RESET

// ── State ─────────────────────────────────────────────────────
typedef enum {
    STEPPER_IDLE    = 0,
    STEPPER_MOVING  = 1
} StepperState;

// ── Public Function Declarations ─────────────────────────────
void Stepper_Init(TIM_HandleTypeDef *htim);
void Stepper_Enable(void);
void Stepper_Disable(void);
void Stepper_Stop(void);
void Stepper_SetDirection(GPIO_PinState direction);
void Stepper_MoveSteps(uint32_t steps, uint32_t step_delay_us);
void Stepper_MoveDegs(float degrees, uint32_t step_delay_us);
void Stepper_MoveContinuous(uint32_t step_delay_us);
StepperState Stepper_GetState(void);
void Stepper_TIM_Callback(TIM_HandleTypeDef *htim);

#endif

/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define SERVO_1_Pin GPIO_PIN_0
#define SERVO_1_GPIO_Port GPIOA
#define SERVO_2_Pin GPIO_PIN_1
#define SERVO_2_GPIO_Port GPIOA
#define SERVO_3_Pin GPIO_PIN_2
#define SERVO_3_GPIO_Port GPIOA
#define SERVO_4_Pin GPIO_PIN_3
#define SERVO_4_GPIO_Port GPIOA
#define STEP_Pin GPIO_PIN_4
#define STEP_GPIO_Port GPIOA
#define DIR_Pin GPIO_PIN_5
#define DIR_GPIO_Port GPIOA
#define ENA_Pin GPIO_PIN_6
#define ENA_GPIO_Port GPIOA
#define CURRENT_SENSOR1_Pin GPIO_PIN_7
#define CURRENT_SENSOR1_GPIO_Port GPIOA
#define CURRENT_SENSOR2_Pin GPIO_PIN_0
#define CURRENT_SENSOR2_GPIO_Port GPIOB
#define STEPPER_LIMIT_Pin GPIO_PIN_12
#define STEPPER_LIMIT_GPIO_Port GPIOB
#define STEPPER_LIMIT_EXTI_IRQn EXTI15_10_IRQn
#define GRIPPER_LIMIT_Pin GPIO_PIN_13
#define GRIPPER_LIMIT_GPIO_Port GPIOB
#define GRIPPER_LIMIT_EXTI_IRQn EXTI15_10_IRQn
#define E_STOP_Pin GPIO_PIN_9
#define E_STOP_GPIO_Port GPIOA
#define E_STOP_EXTI_IRQn EXTI9_5_IRQn
#define TOGGLE_PIN_Pin GPIO_PIN_10
#define TOGGLE_PIN_GPIO_Port GPIOA
#define NRF_CSN_Pin GPIO_PIN_15
#define NRF_CSN_GPIO_Port GPIOA
#define NRF_SCK_Pin GPIO_PIN_3
#define NRF_SCK_GPIO_Port GPIOB
#define NRF_MISO_Pin GPIO_PIN_4
#define NRF_MISO_GPIO_Port GPIOB
#define NRF_MOSI_Pin GPIO_PIN_5
#define NRF_MOSI_GPIO_Port GPIOB
#define NRF_CE_Pin GPIO_PIN_6
#define NRF_CE_GPIO_Port GPIOB
#define NRF_IRQ_Pin GPIO_PIN_7
#define NRF_IRQ_GPIO_Port GPIOB
#define NRF_IRQ_EXTI_IRQn EXTI9_5_IRQn

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

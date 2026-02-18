/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "stm32h7xx_hal.h"

#include "stm32h7xx_nucleo.h"
#include <stdio.h>

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
// Omni geometry
extern double rw;        // wheel radius (m)
extern double L;         // distance from robot center to wheel contact line (m)
extern const double pion180;

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
#define M1curr_Pin GPIO_PIN_0
#define M1curr_GPIO_Port GPIOC
#define PWM1_Pin GPIO_PIN_0
#define PWM1_GPIO_Port GPIOA
#define IMU_RX_Pin GPIO_PIN_3
#define IMU_RX_GPIO_Port GPIOA
#define M2curr_Pin GPIO_PIN_4
#define M2curr_GPIO_Port GPIOA
#define M3curr_Pin GPIO_PIN_5
#define M3curr_GPIO_Port GPIOA
#define ENC1_1_Pin GPIO_PIN_6
#define ENC1_1_GPIO_Port GPIOA
#define BATT_V_Pin GPIO_PIN_1
#define BATT_V_GPIO_Port GPIOB
#define IMU_INT_PIN_Pin GPIO_PIN_15
#define IMU_INT_PIN_GPIO_Port GPIOE
#define IMU_INT_PIN_EXTI_IRQn EXTI15_10_IRQn
#define PWM3_Pin GPIO_PIN_10
#define PWM3_GPIO_Port GPIOB
#define BT_RX_Pin GPIO_PIN_15
#define BT_RX_GPIO_Port GPIOB
#define ENC2_1_Pin GPIO_PIN_12
#define ENC2_1_GPIO_Port GPIOD
#define ENC2_2_Pin GPIO_PIN_13
#define ENC2_2_GPIO_Port GPIOD
#define ENC3_1_Pin GPIO_PIN_6
#define ENC3_1_GPIO_Port GPIOC
#define ENC3_2_Pin GPIO_PIN_7
#define ENC3_2_GPIO_Port GPIOC
#define IMU_TX_Pin GPIO_PIN_5
#define IMU_TX_GPIO_Port GPIOD
#define PWM2_Pin GPIO_PIN_3
#define PWM2_GPIO_Port GPIOB
#define ENC1_2_Pin GPIO_PIN_5
#define ENC1_2_GPIO_Port GPIOB
#define BT_TX_Pin GPIO_PIN_6
#define BT_TX_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
// MD20A motor driver direction pins
#define DIR1_Pin GPIO_PIN_10
#define DIR1_GPIO_Port GPIOE
#define DIR2_Pin GPIO_PIN_11
#define DIR2_GPIO_Port GPIOE
#define DIR3_Pin GPIO_PIN_12
#define DIR3_GPIO_Port GPIOE

// Pololu HP MOSFET ON pins (motor power enable)
#define MOTOR_ON1_Pin GPIO_PIN_8
#define MOTOR_ON1_GPIO_Port GPIOG
#define MOTOR_ON2_Pin GPIO_PIN_9
#define MOTOR_ON2_GPIO_Port GPIOG
#define MOTOR_ON3_Pin GPIO_PIN_10
#define MOTOR_ON3_GPIO_Port GPIOG
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

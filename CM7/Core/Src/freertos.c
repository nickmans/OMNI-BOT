/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
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

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/**
  * @brief  Application Malloc failed hook
  * @param  None
  * @retval None
  */
void vApplicationMallocFailedHook(void)
{
  printf("\r\n\r\n*** MALLOC FAILED ***\r\n");
  printf("FreeRTOS heap exhausted!\r\n");
  printf("Current heap size: %lu bytes\r\n", (unsigned long)configTOTAL_HEAP_SIZE);
  printf("Increase configTOTAL_HEAP_SIZE in FreeRTOSConfig.h\r\n");
  printf("System halted.\r\n\r\n");
  taskDISABLE_INTERRUPTS();
  for(;;);
}

/**
  * @brief  Application stack overflow hook
  * @param  xTask: Task handle
  * @param  pcTaskName: Task name
  * @retval None
  */
void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
  (void)xTask;
  printf("\r\n\r\n*** STACK OVERFLOW ***\r\n");
  printf("Task: %s\r\n", pcTaskName);
  printf("Increase stack size for this task\r\n");
  printf("System halted.\r\n\r\n");
  taskDISABLE_INTERRUPTS();
  for(;;);
}

/* USER CODE END Application */


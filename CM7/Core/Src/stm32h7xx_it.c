/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32h7xx_it.c
  * @brief   Interrupt Service Routines.
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
#include "main.h"
#include "stm32h7xx_it.h"
#include "FreeRTOS.h"
#include "task.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */
static void HardFault_HandlerC(uint32_t *stacked_regs, uint32_t exc_lr);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern ETH_HandleTypeDef heth;
extern DMA_HandleTypeDef hdma_adc1;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
/* USER CODE BEGIN EV */
extern ETH_HandleTypeDef heth;
/* USER CODE END EV */

/******************************************************************************/
/*           Cortex Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
   while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
__attribute__((naked)) void HardFault_Handler(void)
{
  __asm volatile
  (
    "tst lr, #4                         \n"
    "ite eq                             \n"
    "mrseq r0, msp                      \n"
    "mrsne r0, psp                      \n"
    "mov r1, lr                         \n"
    "b HardFault_HandlerC               \n"
  );
}

static void HardFault_HandlerC(uint32_t *stacked_regs, uint32_t exc_lr)
{
  uint32_t cfsr = SCB->CFSR;
  uint32_t hfsr = SCB->HFSR;
  uint32_t bfar = SCB->BFAR;
  uint32_t mmfar = SCB->MMFAR;
  uint32_t afsr = SCB->AFSR;

  uint32_t stacked_r0 = stacked_regs[0];
  uint32_t stacked_r1 = stacked_regs[1];
  uint32_t stacked_r2 = stacked_regs[2];
  uint32_t stacked_r3 = stacked_regs[3];
  uint32_t stacked_r12 = stacked_regs[4];
  uint32_t stacked_lr = stacked_regs[5];
  uint32_t stacked_pc = stacked_regs[6];
  uint32_t stacked_psr = stacked_regs[7];

  const char *task_name = "no_task";
  TaskHandle_t current_task = xTaskGetCurrentTaskHandle();
  if (current_task != NULL)
  {
    task_name = pcTaskGetName(current_task);
  }

  printf("\r\n*** HARDFAULT ***\r\n");
  printf("task=%s\r\n", task_name);
  printf("CFSR=0x%08lX HFSR=0x%08lX AFSR=0x%08lX\r\n",
         (unsigned long)cfsr,
         (unsigned long)hfsr,
         (unsigned long)afsr);
  printf("BFAR=0x%08lX MMFAR=0x%08lX\r\n",
         (unsigned long)bfar,
         (unsigned long)mmfar);
  printf("stack_ptr=0x%08lX EXC_LR=0x%08lX\r\n",
         (unsigned long)stacked_regs,
         (unsigned long)exc_lr);
  printf("R0=0x%08lX R1=0x%08lX R2=0x%08lX R3=0x%08lX R12=0x%08lX\r\n",
         (unsigned long)stacked_r0,
         (unsigned long)stacked_r1,
         (unsigned long)stacked_r2,
         (unsigned long)stacked_r3,
         (unsigned long)stacked_r12);
  printf("LR=0x%08lX PC=0x%08lX PSR=0x%08lX\r\n",
         (unsigned long)stacked_lr,
         (unsigned long)stacked_pc,
         (unsigned long)stacked_psr);

  if (heth.Instance != NULL)
  {
    printf("ETH_DMACSR=0x%08lX\r\n", (unsigned long)heth.Instance->DMACSR);
  }

  while (1)
  {
  }
}

#if 0
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */
  uint32_t cfsr = SCB->CFSR;
  uint32_t hfsr = SCB->HFSR;
  uint32_t bfar = SCB->BFAR;
  uint32_t mmfar = SCB->MMFAR;
  uint32_t afsr = SCB->AFSR;
  uint32_t msp = __get_MSP();

  uint32_t stacked_r0 = *((uint32_t *)(msp + 0u));
  uint32_t stacked_r1 = *((uint32_t *)(msp + 4u));
  uint32_t stacked_r2 = *((uint32_t *)(msp + 8u));
  uint32_t stacked_r3 = *((uint32_t *)(msp + 12u));
  uint32_t stacked_r12 = *((uint32_t *)(msp + 16u));
  uint32_t stacked_lr = *((uint32_t *)(msp + 20u));
  uint32_t stacked_pc = *((uint32_t *)(msp + 24u));
  uint32_t stacked_psr = *((uint32_t *)(msp + 28u));

  const char *task_name = "no_task";
  TaskHandle_t current_task = xTaskGetCurrentTaskHandle();
  if (current_task != NULL)
  {
    task_name = pcTaskGetName(current_task);
  }

  printf("\r\n*** HARDFAULT ***\r\n");
  printf("task=%s\r\n", task_name);
  printf("CFSR=0x%08lX HFSR=0x%08lX AFSR=0x%08lX\r\n",
         (unsigned long)cfsr,
         (unsigned long)hfsr,
         (unsigned long)afsr);
  printf("BFAR=0x%08lX MMFAR=0x%08lX\r\n",
         (unsigned long)bfar,
         (unsigned long)mmfar);
    printf("MSP=0x%08lX\r\n", (unsigned long)msp);
    printf("R0=0x%08lX R1=0x%08lX R2=0x%08lX R3=0x%08lX R12=0x%08lX\r\n",
      (unsigned long)stacked_r0,
      (unsigned long)stacked_r1,
      (unsigned long)stacked_r2,
      (unsigned long)stacked_r3,
      (unsigned long)stacked_r12);
    printf("LR=0x%08lX PC=0x%08lX PSR=0x%08lX\r\n",
      (unsigned long)stacked_lr,
      (unsigned long)stacked_pc,
      (unsigned long)stacked_psr);

    if (heth.Instance != NULL)
    {
      printf("ETH_DMACSR=0x%08lX\r\n", (unsigned long)heth.Instance->DMACSR);
    }

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}
#endif

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Pre-fetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
#if (INCLUDE_xTaskGetSchedulerState == 1 )
  if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED)
  {
#endif /* INCLUDE_xTaskGetSchedulerState */
  xPortSysTickHandler();
#if (INCLUDE_xTaskGetSchedulerState == 1 )
  }
#endif /* INCLUDE_xTaskGetSchedulerState */
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32H7xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32h7xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles DMA1 stream0 global interrupt.
  */
void DMA1_Stream0_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream0_IRQn 0 */

  /* USER CODE END DMA1_Stream0_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_adc1);
  /* USER CODE BEGIN DMA1_Stream0_IRQn 1 */

  /* USER CODE END DMA1_Stream0_IRQn 1 */
}

/**
  * @brief This function handles USART1 global interrupt.
  */
void USART1_IRQHandler(void)
{
  /* USER CODE BEGIN USART1_IRQn 0 */

  /* USER CODE END USART1_IRQn 0 */
  HAL_UART_IRQHandler(&huart1);
  /* USER CODE BEGIN USART1_IRQn 1 */

  /* USER CODE END USART1_IRQn 1 */
}

/**
  * @brief This function handles USART2 global interrupt.
  */
void USART2_IRQHandler(void)
{
  /* USER CODE BEGIN USART2_IRQn 0 */

  /* USER CODE END USART2_IRQn 0 */
  HAL_UART_IRQHandler(&huart2);
  /* USER CODE BEGIN USART2_IRQn 1 */

  /* USER CODE END USART2_IRQn 1 */
}

/**
  * @brief This function handles EXTI line[15:10] interrupts.
  */
void EXTI15_10_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI15_10_IRQn 0 */

  /* USER CODE END EXTI15_10_IRQn 0 */
  BSP_PB_IRQHandler(BUTTON_USER);
  HAL_GPIO_EXTI_IRQHandler(IMU_INT_PIN_Pin);
  /* USER CODE BEGIN EXTI15_10_IRQn 1 */

  /* USER CODE END EXTI15_10_IRQn 1 */
}

/**
  * @brief This function handles Ethernet global interrupt.
  */
void ETH_IRQHandler(void)
{
  /* USER CODE BEGIN ETH_IRQn 0 */

  /* USER CODE END ETH_IRQn 0 */
  HAL_ETH_IRQHandler(&heth);
  /* USER CODE BEGIN ETH_IRQn 1 */

  /* USER CODE END ETH_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

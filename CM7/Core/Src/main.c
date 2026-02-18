/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "cmsis_os.h"
#include "lwip.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "controller.h"
#include "sensor.h"
#include <stdio.h>
#include "cmd.h"
#include "IMU.h"
#include "udp_client.h"
#include "battery_monitor.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
// Omni geometry
double rw = 0.075;       // wheel radius (m)
double L = 0.2;          // distance from robot center to wheel contact line (m)
static const double CPR = SATURN5303_OUTPUT_CPR;
const int32_t CNT_MID = 32768;  // 0x8000
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* DUAL_CORE_BOOT_SYNC_SEQUENCE: Define for dual core boot synchronization    */
/*                             demonstration code based on hardware semaphore */
/* This define is present in both CM7/CM4 projects                            */
/* To comment when developping/debugging on a single core                     */
#define DUAL_CORE_BOOT_SYNC_SEQUENCE

#if defined(DUAL_CORE_BOOT_SYNC_SEQUENCE)
#ifndef HSEM_ID_0
#define HSEM_ID_0 (0U) /* HW semaphore 0*/
#endif
#endif /* DUAL_CORE_BOOT_SYNC_SEQUENCE */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
double const pion180 = 0.01745329251;
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

COM_InitTypeDef BspCOMInit;
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;
DMA_HandleTypeDef hdma_adc1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim8;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* USER CODE BEGIN PV */

static TIM_HandleTypeDef htim6;

osThreadId_t control_id;
const osThreadAttr_t control_att = {
  .name = "control_t",
  .stack_size = 1028*8,
  .priority = (osPriority_t) osPriorityNormal,
};
osThreadId_t remote_id;
const osThreadAttr_t remote_att = {
  .name = "remote_t",
  .stack_size = 1028*6,  // Reduced from 8224 to 6168 to save heap - main loop needs ~2-3KB
  .priority = (osPriority_t) osPriorityNormal,
};
osThreadId_t ethernet_id;
const osThreadAttr_t ethernet_att = {
  .name = "ethernet_t",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityLow,
};

#define MOTOR_CURRENT_DMA_CHANNELS   3u
#define MOTOR_CURRENT_DMA_SEQUENCES  32u
#define MOTOR_CURRENT_DMA_SAMPLES    (MOTOR_CURRENT_DMA_CHANNELS * MOTOR_CURRENT_DMA_SEQUENCES)
static uint16_t s_motorCurrentAdcDmaBuf[MOTOR_CURRENT_DMA_SAMPLES] = {0u};
static uint8_t s_motorCurrentDmaStarted = 0u;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM8_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);
void StartDefaultTask(void *argument);

/* USER CODE BEGIN PFP */
void control(void *argument);
void remote(void *argument);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static uint8_t rx1_byte;
static uint8_t rx2_byte;

static void TIM6_User_Init(void)
{
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 274;
  htim6.Init.Period = 99;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
}

static void UartRearmRxIt(UART_HandleTypeDef *huart, uint8_t *byte)
{
  if ((huart == NULL) || (byte == NULL))
  {
    return;
  }

  if (HAL_UART_Receive_IT(huart, byte, 1) != HAL_OK)
  {
    (void)HAL_UART_AbortReceive(huart);
    (void)HAL_UART_Receive_IT(huart, byte, 1);
  }
}

void UartStartRx(void)
{
  UartRearmRxIt(&huart1, &rx1_byte);
  UartRearmRxIt(&huart2, &rx2_byte);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart == &huart1)
  {
    CMD_OnUartRxByteFromISR(rx1_byte);
    UartRearmRxIt(&huart1, &rx1_byte);
  }
  else if (huart == &huart2)
  {
    BNO_RVC_OnByte(rx2_byte);
    UartRearmRxIt(&huart2, &rx2_byte);
  }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart == &huart1)
  {
    UartRearmRxIt(&huart1, &rx1_byte);
  }
  else if (huart == &huart2)
  {
    UartRearmRxIt(&huart2, &rx2_byte);
  }
}
void TIM_init(void)
{
// Initialize MD20A motor drivers: 0% duty cycle and forward direction
__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);
__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0);
__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 0);

// Set all direction pins to forward (LOW)
HAL_GPIO_WritePin(DIR1_GPIO_Port, DIR1_Pin, GPIO_PIN_RESET);
HAL_GPIO_WritePin(DIR2_GPIO_Port, DIR2_Pin, GPIO_PIN_RESET);
HAL_GPIO_WritePin(DIR3_GPIO_Port, DIR3_Pin, GPIO_PIN_RESET);

HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);

HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
	__HAL_TIM_SET_COUNTER(&htim3, CNT_MID);
HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
	__HAL_TIM_SET_COUNTER(&htim4, CNT_MID);
HAL_TIM_Encoder_Start(&htim8, TIM_CHANNEL_ALL);
	__HAL_TIM_SET_COUNTER(&htim8, CNT_MID);
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */
/* USER CODE BEGIN Boot_Mode_Sequence_0 */
#if defined(DUAL_CORE_BOOT_SYNC_SEQUENCE)
  int32_t timeout;
#endif /* DUAL_CORE_BOOT_SYNC_SEQUENCE */
/* USER CODE END Boot_Mode_Sequence_0 */

/* USER CODE BEGIN Boot_Mode_Sequence_1 */
#if defined(DUAL_CORE_BOOT_SYNC_SEQUENCE)
  /* Wait until CPU2 boots and enters in stop mode or timeout*/
  timeout = 0xFFFF;
  while((__HAL_RCC_GET_FLAG(RCC_FLAG_D2CKRDY) != RESET) && (timeout-- > 0));
  if ( timeout < 0 )
  {
  Error_Handler();
  }
#endif /* DUAL_CORE_BOOT_SYNC_SEQUENCE */
/* USER CODE END Boot_Mode_Sequence_1 */
  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* Configure the peripherals common clocks */
  PeriphCommonClock_Config();
/* USER CODE BEGIN Boot_Mode_Sequence_2 */
#if defined(DUAL_CORE_BOOT_SYNC_SEQUENCE)
/* When system initialization is finished, Cortex-M7 will release Cortex-M4 by means of
HSEM notification */
/*HW semaphore Clock enable*/
__HAL_RCC_HSEM_CLK_ENABLE();
/*Take HSEM */
HAL_HSEM_FastTake(HSEM_ID_0);
/*Release HSEM in order to notify the CPU2(CM4)*/
HAL_HSEM_Release(HSEM_ID_0,0);
/* wait until CPU2 wakes up from stop mode */
timeout = 0xFFFF;
while((__HAL_RCC_GET_FLAG(RCC_FLAG_D2CKRDY) == RESET) && (timeout-- > 0));
if ( timeout < 0 )
{
Error_Handler();
}
#endif /* DUAL_CORE_BOOT_SYNC_SEQUENCE */
/* USER CODE END Boot_Mode_Sequence_2 */

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  MX_TIM4_Init();
  MX_TIM8_Init();
  MX_TIM3_Init();
  MX_USART2_UART_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  /* USER CODE BEGIN 2 */
  TIM6_User_Init();
  BatteryMonitor_Init();
  if (HAL_ADC_Start_DMA(&hadc1, (uint32_t *)s_motorCurrentAdcDmaBuf, MOTOR_CURRENT_DMA_SAMPLES) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_Base_Start(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  s_motorCurrentDmaStarted = 1u;
  CMD_Init(&huart1);
  BNO_RVC_Init();
  UartStartRx();
  TIM_init();
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  CMD_StartTask();
  remote_id = osThreadNew(remote, NULL, &remote_att);
  ethernet_id = osThreadNew(UDP_Client_Task, NULL, &ethernet_att);
  //control_id = osThreadNew(control, NULL, &control_att);
  //imu_id = osThreadNew(imu_task, NULL, &imu_att);

  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Initialize leds */
  BSP_LED_Init(LED_GREEN);
  BSP_LED_Init(LED_YELLOW);
  BSP_LED_Init(LED_RED);

  /* Initialize USER push-button, will be used to trigger an interrupt each time it's pressed.*/
  BSP_PB_Init(BUTTON_USER, BUTTON_MODE_EXTI);

  /* Initialize COM1 port (115200, 8 bits (7-bit data + 1 stop bit), no parity */
  BspCOMInit.BaudRate   = 115200;
  BspCOMInit.WordLength = COM_WORDLENGTH_8B;
  BspCOMInit.StopBits   = COM_STOPBITS_1;
  BspCOMInit.Parity     = COM_PARITY_NONE;
  BspCOMInit.HwFlowCtl  = COM_HWCONTROL_NONE;
  if (BSP_COM_Init(COM1, &BspCOMInit) != BSP_ERROR_NONE)
  {
    Error_Handler();
  }

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_DIRECT_SMPS_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 5;
  RCC_OscInitStruct.PLL.PLLN = 46;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 5;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_CKPER;
  PeriphClkInitStruct.CkperClockSelection = RCC_CLKPSOURCE_HSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 3;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIG_T6_TRGO;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DR;
  hadc1.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  hadc1.Init.LeftBitShift = ADC_LEFTBITSHIFT_NONE;
  hadc1.Init.OversamplingMode = DISABLE;
  hadc1.Init.Oversampling.Ratio = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_10;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_64CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  sConfig.OffsetSignedSaturation = DISABLE;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_18;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_19;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */

  /** Common config
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV2;
  hadc2.Init.Resolution = ADC_RESOLUTION_16B;
  hadc2.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc2.Init.LowPowerAutoWait = DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DR;
  hadc2.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc2.Init.LeftBitShift = ADC_LEFTBITSHIFT_NONE;
  hadc2.Init.OversamplingMode = DISABLE;
  hadc2.Init.Oversampling.Ratio = 1;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_64CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  sConfig.OffsetSignedSaturation = DISABLE;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 7371;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 4;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 4;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 4;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 4;
  if (HAL_TIM_Encoder_Init(&htim4, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief TIM8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM8_Init(void)
{

  /* USER CODE BEGIN TIM8_Init 0 */

  /* USER CODE END TIM8_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 0;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 65535;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 4;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 4;
  if (HAL_TIM_Encoder_Init(&htim8, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin : IMU_INT_PIN_Pin */
  GPIO_InitStruct.Pin = IMU_INT_PIN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(IMU_INT_PIN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PA8 PA11 PA12 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_11|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF10_OTG1_FS;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  // Initialize MD20A direction pins as outputs (default LOW for forward)
  GPIO_InitStruct.Pin = DIR1_Pin | DIR2_Pin | DIR3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(DIR1_GPIO_Port, &GPIO_InitStruct);

  // Initialize Pololu HP MOSFET ON pins as outputs (default LOW)
  GPIO_InitStruct.Pin = MOTOR_ON1_Pin | MOTOR_ON2_Pin | MOTOR_ON3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  HAL_GPIO_WritePin(GPIOG, MOTOR_ON1_Pin | MOTOR_ON2_Pin | MOTOR_ON3_Pin, GPIO_PIN_RESET);
  
  // Set all to forward initially (LOW)
  HAL_GPIO_WritePin(DIR1_GPIO_Port, DIR1_Pin | DIR2_Pin | DIR3_Pin, GPIO_PIN_RESET);

  // Staggered motor ON enable sequence: HIGH with 50ms delay between each
  HAL_GPIO_WritePin(MOTOR_ON1_GPIO_Port, MOTOR_ON1_Pin, GPIO_PIN_SET);
  HAL_Delay(50);
  HAL_GPIO_WritePin(MOTOR_ON2_GPIO_Port, MOTOR_ON2_Pin, GPIO_PIN_SET);
  HAL_Delay(50);
  HAL_GPIO_WritePin(MOTOR_ON3_GPIO_Port, MOTOR_ON3_Pin, GPIO_PIN_SET);
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
double yaw_shifter = 0;
static double twopion60 = 2*M_PI/60;
void BSP_PB_Callback(Button_TypeDef Button)
{
	static int on = 0;
	if (on == 0)
	{
		// speed[0] = 3;
		// speed[1] = 0;
		// speed[2] = 0;
    yaw_shifter = 15*twopion60;
		on = 1;
	}
	else if (on == 1)
	{
		// speed[0] = 0;
		// speed[1] = 3;
		// speed[2] = 0;
    yaw_shifter = 30*twopion60;
		on = 2;
	}
	else if (on == 2)
	{
		// speed[0] = 0;
		// speed[1] = 0;
		// speed[2] = 3;
    yaw_shifter = 45*twopion60;
		on = 3;
	}
  else if (on == 3)
	{
		// speed[0] = 0;
		// speed[1] = 0;
		// speed[2] = 0;
    yaw_shifter = 0;
		on = 0;
	}
	/*if (on == 0)
	{
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, SLOW);
		on = 1;
	}
	else if (on == 1)
	{
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, MED);
		on = 2;
	}
	else if (on == 2)
	{
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, MED1);
		on = 3;
	}
	else if (on == 3)
	{
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, MED2);
		on = 4;
	}
	else if (on == 4)
	{
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, MED3);
		on = 5;
	}
	else if (on == 5)
	{
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, MAXf);
		on = 6;
	}
	else if (on == 6)
	{
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, NEUTRAL);
		on = 0;
	}*/


	BSP_LED_Toggle(LED_RED);
}
double x[3] = {0},xd[5] = {0},u_out[3] = {0};
double vd[3] = {0};
static void enc(double dt, double rpm[3])
{
	int32_t cnt1	 = (int32_t)__HAL_TIM_GET_COUNTER(&htim3);
	int32_t delta1 = cnt1 - CNT_MID;
	__HAL_TIM_SET_COUNTER(&htim3, CNT_MID);

	int32_t cnt2	 = (int32_t)__HAL_TIM_GET_COUNTER(&htim4);
	int32_t delta2 = cnt2 - CNT_MID;
	__HAL_TIM_SET_COUNTER(&htim4, CNT_MID);

	int32_t cnt3	 = (int32_t)__HAL_TIM_GET_COUNTER(&htim8);
	int32_t delta3 = cnt3 - CNT_MID;
	__HAL_TIM_SET_COUNTER(&htim8, CNT_MID);

  static double t_since_edge_s[3] = {0.0, 0.0, 0.0};
  static double rpm_period[3] = {0.0, 0.0, 0.0};
  static double rpm_filt[3] = {0.0, 0.0, 0.0};

  const int32_t delta[3] = {delta1, delta2, delta3};
  const double rpm_per_count = 60.0 / (CPR * dt);
  const double pulse_timeout_s = 0.25;
  const double alpha = 0.35;

  for (int i = 0; i < 3; ++i)
  {
    t_since_edge_s[i] += dt;

    const int32_t d = delta[i];
    double rpm_window = (double)d * rpm_per_count;
    double rpm_meas = rpm_window;

    if (d != 0)
    {
      const int32_t mag = (d >= 0) ? d : -d;
      if (t_since_edge_s[i] > 0.0)
      {
        rpm_period[i] = ((double)d * 60.0) / (CPR * t_since_edge_s[i]);
      }
      t_since_edge_s[i] = 0.0;

      if (mag <= 1)
      {
        const double blend_period = 0.85;
        rpm_meas = (blend_period * rpm_period[i]) + ((1.0 - blend_period) * rpm_window);
      }
      else
      {
        const double blend_period = 0.35;
        rpm_meas = (blend_period * rpm_period[i]) + ((1.0 - blend_period) * rpm_window);
      }
    }
    else
    {
      if (t_since_edge_s[i] > pulse_timeout_s)
      {
        rpm_period[i] = 0.0;
      }
      rpm_meas = rpm_period[i];
    }

    rpm_filt[i] += alpha * (rpm_meas - rpm_filt[i]);
    rpm[i] = rpm_filt[i];
  }

}

static HAL_StatusTypeDef ReadMotorCurrentAdcRaw(uint16_t adc_raw_out[3])
{
  if (s_motorCurrentDmaStarted == 0u)
  {
    return HAL_ERROR;
  }

  uint32_t accum[3] = {0u, 0u, 0u};
  for (uint16_t i = 0u; i < MOTOR_CURRENT_DMA_SAMPLES; i += 3u)
  {
    accum[0] += s_motorCurrentAdcDmaBuf[i + 0u];
    accum[1] += s_motorCurrentAdcDmaBuf[i + 1u];
    accum[2] += s_motorCurrentAdcDmaBuf[i + 2u];
  }

  adc_raw_out[0] = (uint16_t)(accum[0] / MOTOR_CURRENT_DMA_SEQUENCES);
  adc_raw_out[1] = (uint16_t)(accum[1] / MOTOR_CURRENT_DMA_SEQUENCES);
  adc_raw_out[2] = (uint16_t)(accum[2] / MOTOR_CURRENT_DMA_SEQUENCES);
  return HAL_OK;
}

static const float kMotorCurrentAdcVref = 3.3f;
static const float kMotorCurrentAdcFullScale = 4095.0f;
static const float kMotorCurrentSensitivityVPerA[3] = {0.185f, 0.185f, 0.185f};
static float s_motorCurrentOffsetRaw[3] = {2048.0f, 2048.0f, 2048.0f};
static float s_motorCurrentBiasA[3] = {0.0f, 0.0f, 0.0f};

static void CalibrateMotorCurrentOffset(uint16_t sample_count)
{
  if (sample_count == 0u)
  {
    return;
  }

  uint32_t accum[3] = {0u, 0u, 0u};
  uint16_t adc_raw[3] = {0u, 0u, 0u};
  uint16_t valid_samples = 0u;

  for (uint16_t i = 0u; i < sample_count; ++i)
  {
    if (ReadMotorCurrentAdcRaw(adc_raw) == HAL_OK)
    {
      accum[0] += adc_raw[0];
      accum[1] += adc_raw[1];
      accum[2] += adc_raw[2];
      ++valid_samples;
    }

    vTaskDelay(pdMS_TO_TICKS(2));
  }

  if (valid_samples > 0u)
  {
    s_motorCurrentOffsetRaw[0] = (float)accum[0] / (float)valid_samples;
    s_motorCurrentOffsetRaw[1] = (float)accum[1] / (float)valid_samples;
    s_motorCurrentOffsetRaw[2] = (float)accum[2] / (float)valid_samples;
  }
}

static float MotorCurrentRawToAmps(uint16_t raw, uint8_t channel_index)
{
  uint8_t idx = channel_index;
  if (idx > 2u)
  {
    idx = 2u;
  }

  const float adc_lsb_v = kMotorCurrentAdcVref / kMotorCurrentAdcFullScale;
  const float delta_v = ((float)raw - s_motorCurrentOffsetRaw[idx]) * adc_lsb_v;
  return delta_v / kMotorCurrentSensitivityVPerA[idx];
}

#include "math.h"
double yaw = 0,yawrate = 0,ax = 0,ay = 0,az = 0;
static int counter = 0;
double battery_v = 24;
void remote(void *argument)
{
	int tick = 10;
	const TickType_t period = pdMS_TO_TICKS(tick); // 100 Hz
	TickType_t lastWakeTime = xTaskGetTickCount();
  const TickType_t posePeriod = pdMS_TO_TICKS(200); // 5 Hz pose heartbeat
  TickType_t lastPoseTick = xTaskGetTickCount();
  const TickType_t rpmPrintPeriod = pdMS_TO_TICKS(200);
  TickType_t lastRpmPrintTick = xTaskGetTickCount();
  const TickType_t batteryPeriod = pdMS_TO_TICKS(10000);
  TickType_t lastBatteryTick = xTaskGetTickCount();

    const float dt = (float)(period * portTICK_PERIOD_MS) * 1e-3f;
	
    //CMD_Send("Remote task started\n");
    CalibrateMotorCurrentOffset(200u);
	uint16_t loop_cnt = 0;
	for (;;)
	{
	    vTaskDelayUntil(&lastWakeTime, period);
	    
	    static double rpm[3] = {0};
      static uint16_t current_adc_raw[3] = {0u, 0u, 0u};
      static float current_a[3] = {0.0f, 0.0f, 0.0f};

	    // Get IMU data with linear acceleration (gravity compensated)
	    BNO_RVC_UpdateMain(&yaw, &yawrate, &ax, &ay, &az);

	    enc(dt,rpm);

      TickType_t nowTick = xTaskGetTickCount();
      if ((nowTick - lastBatteryTick) >= batteryPeriod)
      {
        battery_v = (double)BatteryMonitor_ReadVoltage_V();
        const double battery_scaled_max_rpm = (2.5 * battery_v) + 70.0;
        PWM_SetMaxRpmLimit(battery_scaled_max_rpm);
        lastBatteryTick = nowTick;
        //printf(" Battery: %.2f V \n",battery_v);
        char battery_msg[48];
        (void)snprintf(battery_msg, sizeof(battery_msg), "Battery: %.2f V\r\n", battery_v);
        CMD_Send(battery_msg);
        if (battery_v < 21) 
        {
          CMD_Send("Charge The Bot !!!\n");
        }
      }

      if ((nowTick - lastRpmPrintTick) >= rpmPrintPeriod)
      {
        if (ReadMotorCurrentAdcRaw(current_adc_raw) == HAL_OK)
        {
          current_a[0] = MotorCurrentRawToAmps(current_adc_raw[0], 0u);
          current_a[1] = MotorCurrentRawToAmps(current_adc_raw[1], 1u);
          current_a[2] = MotorCurrentRawToAmps(current_adc_raw[2], 2u);
        }

        if (wheel_test_mode)
        {
          int idx = (wheel_test_index >= 0 && wheel_test_index < 3) ? wheel_test_index : 0;
           double target_rpm = wheel_test_target_rpm;
           double measured_rpm = rpm[idx];
           double err_rpm = target_rpm - measured_rpm;
          printf("WTEST w%d target=%.2f rpm measured=%.2f rpm err=%.2f\r\n",
                 idx + 1,
                 target_rpm,
             measured_rpm,
                 err_rpm,
                 (double)current_a[0],
                 (double)current_a[1],
                 (double)current_a[2]);
        }
        else
        {
          /*printf("RPM: %.2f %.2f %.2f Heading: %.2f \r\n",
                 rpm[0],
                 rpm[1],
                 rpm[2],
				 yaw);*/
        }
        lastRpmPrintTick = nowTick;
      }
	    
      static uint8_t last_traj_mode = 0u;
      const uint8_t cur_traj_mode = traj_mode;

      const double yaw_rad = yaw * pion180 + yaw_shifter;

      // Rising edge: enter trajectory-follow mode
      if ((last_traj_mode == 0u) && (cur_traj_mode == 1u))
      {
          StateEstimator_Reset(0.0, 0.0, 0.0);
          StateEstimator_ZeroImuYaw(yaw_rad);
          // Clear any stale desired state
          for (int i = 0; i < 5; i++) { xd[i] = 0.0; }
      }
      last_traj_mode = cur_traj_mode;

      // Always run the state estimator so pose is available in both modes.
      // Convert measured wheel speeds (rpm) to rad/s for estimator.
      double w_rad_s[3];
      w_rad_s[0] = rpm[0] * twopion60;
      w_rad_s[1] = rpm[1] * twopion60;
      w_rad_s[2] = rpm[2] * twopion60;
      StateEstimator_Update(w_rad_s, (double)dt, yaw_rad, x);

        if (wheel_test_mode)
        {
          int idx = (wheel_test_index >= 0 && wheel_test_index < 3) ? wheel_test_index : 0;

          speed[0] = 0.0;
          speed[1] = 0.0;
          speed[2] = 0.0;
          speed[idx] = (WHEEL_TEST_CMD_SIGN * wheel_test_target_rpm) * twopion60;
        }
        else if (cur_traj_mode == 0u)
      {
          // =====================
          // traj 0: remote/manual
          // =====================
          vd[0] = vxd;
          vd[1] = vyd;
          vd[2] = yawrated;
          Controller_Step(x, xd, vd, 0, dt);  // selector=0 for velocity control
          /*static int countter = 0;
          static float sum1 = 0.0f, sum2 = 0.0f, sum3 = 0.0f;

          sum1 += current_a[0];
          sum2 += current_a[1];
          sum3 += current_a[2];
          countter++;

          if (countter >= 50) {
              printf("%.3f %.3f %.3f\n", sum1 / countter, sum2 / countter, sum3 / countter);
              countter = 0;
              sum1 = sum2 = sum3 = 0.0f;
          }*/
      }
      else
      {
          // =========================
          // traj 1: follow Pi5 traj
          // =========================
          // Pull latest trajectory (if any)
          typedef struct __attribute__((packed))
          {
              float x;
              float y;
              float yaw;
              float velocity;
          } Pi5TrajectoryKnot;

          static Pi5TrajectoryKnot knots[UDP_MAX_TRAJ_KNOTS];
          uint16_t n_knots = 0;
          float knot_dt = 0.0f;
          uint16_t traj_flags = 0;
          uint32_t traj_t0_ms = 0;
          uint32_t traj_seq = 0;

          bool have_traj = UDP_Client_CopyLatestTraj(knots,
                                                    sizeof(knots),
                                                    &n_knots,
                                                    &knot_dt,
                                                    &traj_flags,
                                                    &traj_t0_ms,
                                                    &traj_seq);

          // Default: 0'd / hold trajectory (no motion) until a valid traj arrives
          xd[0] = 0.0;
          xd[1] = 0.0;
          xd[2] = 0.0;
          xd[3] = 0.0;
          xd[4] = 0.0;

          if (have_traj && (n_knots > 0u) && (knot_dt > 0.0f))
          {
              uint32_t now_ms = HAL_GetTick();
              uint32_t dt_ms = (uint32_t)(knot_dt * 1000.0f);
              if (dt_ms == 0u) { dt_ms = 1u; }

              // Select knot by time since trajectory start
              uint32_t elapsed_ms = (now_ms >= traj_t0_ms) ? (now_ms - traj_t0_ms) : 0u;
              uint32_t idx = elapsed_ms / dt_ms;
              if (idx >= (uint32_t)n_knots) { idx = (uint32_t)n_knots - 1u; }

              const Pi5TrajectoryKnot *k = &knots[idx];
              xd[0] = (double)k->x;
              xd[1] = (double)k->y;
              xd[2] = (double)k->yaw;

              // Feedforward velocity in world frame
              if ((idx + 1u) < (uint32_t)n_knots)
              {
                  const Pi5TrajectoryKnot *k2 = &knots[idx + 1u];
                  const double dtx = (double)knot_dt;
                  xd[3] = ((double)k2->x - (double)k->x) / dtx;
                  xd[4] = ((double)k2->y - (double)k->y) / dtx;
              }
              else
              {
                  const double c = cos((double)k->yaw);
                  const double s = sin((double)k->yaw);
                  xd[3] = (double)k->velocity * c;
                  xd[4] = (double)k->velocity * s;
              }
          }

          // Use trajectory mode selector=1
          Controller_Step(x, xd, vd, 1, dt);
      }

      PWM(rpm,dt);

      // Push pose to Pi5 at 5 Hz to keep link alive and avoid stale data
      if ((nowTick - lastPoseTick) >= posePeriod)
      {
    	    PosePayload pose;
    	    pose.pose_t_ms = HAL_GetTick();
          // Publish estimated pose in both manual and trajectory modes.
          pose.x = (float)x[0];
          pose.y = (float)x[1];
          pose.yaw = (float)x[2];
    	    pose.vx = 0.0f;
    	    pose.vy = 0.0f;
    	    pose.wz = 0.0f;

    	    queue_pose(&pose);

          /*char imu_msg[96];
          (void)snprintf(imu_msg,
                         sizeof(imu_msg),
                         "imu yaw=%.2f yawrate=%.4f\r\n",
                         yaw,
                         yawrate);
          CMD_Send(imu_msg);*/
		  //UDP_Client_SendZeroPose();
    	  lastPoseTick = nowTick;
      }


	    //if (counter++%100 == 0)
	    {
	    	//printf("%.2f %.2f %.2f\n", ax,ay,az);
	    }

	}
}

void control(void *argument)
{
	//uint32_t counter = 0;
    const TickType_t period = pdMS_TO_TICKS(10);   // 100 Hz
    TickType_t lastWakeTime = xTaskGetTickCount(); // init once
	for(;;)
	{
		//uint32_t t_last = HAL_GetTick();

		//Controller_Step(x,xd,u_out,vd,1);

		//uint32_t c_t = HAL_GetTick() - t_last;
		//printf("%lu\n",c_t);

		//if (counter++%10 == 0)
		{
			//printf("%.4f %.4f %.4f\n",u_out[0],u_out[1],u_out[2]);
		}

		vTaskDelayUntil(&lastWakeTime, period);
	}
}

int _write(int file, char *ptr, int len) {
    HAL_UART_Transmit(&hcom_uart[COM1], (uint8_t*)ptr, len, 50);
    return len;
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* init code for LWIP */
  MX_LWIP_Init();
  /* USER CODE BEGIN 5 */
   for(;;)
  {

	  BSP_LED_Toggle(LED_GREEN);
	  osDelay(333);
  }
  /* USER CODE END 5 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdlib.h>
#include <algorithm>
#include <cstdint>
#include <stdbool.h>
#include <math.h>

#include "interfaces/AppsAbstract.hpp"
#include "interfaces/BrakesAbstract.hpp"
#include "interfaces/AnalogsAbstract.hpp"
#include "interfaces/ScAbstract.hpp"
#include "interfaces/AccelerometerAbstract.hpp"
#include "PUTM_EV_CAN_LIBRARY_2024/lib/can_interface.hpp"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

uint16_t adc1_dma_buffer[200];
uint16_t adc2_dma_buffer[250];

bool rtd {false};
bool inverterStatus {false};

Apps apps;
Brakes brakes;
Analog analogs;
Accelerometer accelerometer;
SC sc;

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;
DMA_HandleTypeDef hdma_adc1;
DMA_HandleTypeDef hdma_adc2;

DAC_HandleTypeDef hdac1;
DAC_HandleTypeDef hdac2;

FDCAN_HandleTypeDef hfdcan1;
FDCAN_HandleTypeDef hfdcan2;

IWDG_HandleTypeDef hiwdg;

TIM_HandleTypeDef htim2;

/* Definitions for MainTask */
osThreadId_t MainTaskHandle;
const osThreadAttr_t MainTask_attributes = {
  .name = "MainTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal
};
/* Definitions for BlinkTask */
osThreadId_t BlinkTaskHandle;
const osThreadAttr_t BlinkTask_attributes = {
  .name = "BlinkTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow
};
/* Definitions for AmkTask */
osThreadId_t AmkTaskHandle;
const osThreadAttr_t AmkTask_attributes = {
  .name = "AmkTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow
};
/* USER CODE BEGIN PV */
enum struct StateMachine {
	UNDEFINED = -1,
	ERROR_RESET,
	IDLING,
	STARTUP,
	TORQUE_CONTROL,
	SWITCH_OFF
};
StateMachine state = StateMachine::UNDEFINED;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);
static void MX_DAC1_Init(void);
static void MX_DAC2_Init(void);
static void MX_TIM2_Init(void);
static void MX_FDCAN1_Init(void);
static void MX_FDCAN2_Init(void);
static void MX_IWDG_Init(void);
void StartMainTask(void *argument);
void StartBlinkTask(void *argument);
void StartAmkTask(void *argument);

/* USER CODE BEGIN PFP */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	uint8_t j = 0;
	if (hadc->Instance == ADC1)
	{
		int j = 0;
		for(int i = 0; i < 100; i = i+2)
		{
			apps.apps1_val_raw[j] = adc1_dma_buffer[i];
			analogs.steering_position_val_raw[j] = adc1_dma_buffer[i+1];
			j++;
		}
	}
	else
	{
		j = 0;
		for(int i = 0; i < 250; i = i+5)
		{
			apps.apps2_val_raw[j] = adc2_dma_buffer[i+4];
			brakes.brake_pressure_rear_val_raw[j] = adc2_dma_buffer[i];
			brakes.brake_pressure_front_val_raw[j] = adc2_dma_buffer[i+1];
			j++;
		}
	}
}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint16_t apps_value_to_send;
std::pair<uint16_t, uint16_t> brakePressureValueToSend;
int16_t steering_position_to_send;
uint8_t sc_state;
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_DAC1_Init();
  MX_DAC2_Init();
  MX_TIM2_Init();
  MX_FDCAN1_Init();
  MX_FDCAN2_Init();
//  MX_IWDG_Init();
  /* USER CODE BEGIN 2 */

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
  /* creation of MainTask */
  MainTaskHandle = osThreadNew(StartMainTask, NULL, &MainTask_attributes);

  /* creation of BlinkTask */
  BlinkTaskHandle = osThreadNew(StartBlinkTask, NULL, &BlinkTask_attributes);

  /* creation of AmkTask */
//  AmkTaskHandle = osThreadNew(StartAmkTask, NULL, &AmkTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

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

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 20;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
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
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.GainCompensation = 0;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 2;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIG_T2_TRGO;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
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
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_15;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  sConfig.SamplingTime = ADC_SAMPLETIME_6CYCLES_5;
  sConfig.OffsetNumber = ADC_OFFSET_1;
  sConfig.Offset = 1;
  sConfig.OffsetSign = ADC_OFFSET_SIGN_NEGATIVE;
  sConfig.OffsetSaturation = ENABLE;
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
  hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.GainCompensation = 0;
  hadc2.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc2.Init.LowPowerAutoWait = DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.NbrOfConversion = 5;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConv = ADC_EXTERNALTRIG_T2_TRGO;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc2.Init.DMAContinuousRequests = ENABLE;
  hadc2.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc2.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  sConfig.OffsetNumber = ADC_OFFSET_1;
  sConfig.Offset = 1;
  sConfig.OffsetSign = ADC_OFFSET_SIGN_NEGATIVE;
  sConfig.OffsetSaturation = DISABLE;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_11;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  sConfig.OffsetNumber = ADC_OFFSET_2;
  sConfig.Offset = 2;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_12;
  sConfig.Rank = ADC_REGULAR_RANK_4;
  sConfig.OffsetNumber = ADC_OFFSET_3;
  sConfig.Offset = 3;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_17;
  sConfig.Rank = ADC_REGULAR_RANK_5;
  sConfig.OffsetNumber = ADC_OFFSET_4;
  sConfig.Offset = 4;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

}

/**
  * @brief DAC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC1_Init(void)
{

  /* USER CODE BEGIN DAC1_Init 0 */

  /* USER CODE END DAC1_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC1_Init 1 */

  /* USER CODE END DAC1_Init 1 */

  /** DAC Initialization
  */
  hdac1.Instance = DAC1;
  if (HAL_DAC_Init(&hdac1) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT2 config
  */
  sConfig.DAC_HighFrequency = DAC_HIGH_FREQUENCY_INTERFACE_MODE_AUTOMATIC;
  sConfig.DAC_DMADoubleDataMode = DISABLE;
  sConfig.DAC_SignedFormat = DISABLE;
  sConfig.DAC_SampleAndHold = DAC_SAMPLEANDHOLD_DISABLE;
  sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
  sConfig.DAC_Trigger2 = DAC_TRIGGER_NONE;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  sConfig.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_EXTERNAL;
  sConfig.DAC_UserTrimming = DAC_TRIMMING_FACTORY;
  if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC1_Init 2 */

  /* USER CODE END DAC1_Init 2 */

}

/**
  * @brief DAC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC2_Init(void)
{

  /* USER CODE BEGIN DAC2_Init 0 */

  /* USER CODE END DAC2_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC2_Init 1 */

  /* USER CODE END DAC2_Init 1 */

  /** DAC Initialization
  */
  hdac2.Instance = DAC2;
  if (HAL_DAC_Init(&hdac2) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT1 config
  */
  sConfig.DAC_HighFrequency = DAC_HIGH_FREQUENCY_INTERFACE_MODE_AUTOMATIC;
  sConfig.DAC_DMADoubleDataMode = DISABLE;
  sConfig.DAC_SignedFormat = DISABLE;
  sConfig.DAC_SampleAndHold = DAC_SAMPLEANDHOLD_DISABLE;
  sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
  sConfig.DAC_Trigger2 = DAC_TRIGGER_NONE;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  sConfig.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_EXTERNAL;
  sConfig.DAC_UserTrimming = DAC_TRIMMING_FACTORY;
  if (HAL_DAC_ConfigChannel(&hdac2, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC2_Init 2 */

  /* USER CODE END DAC2_Init 2 */

}

/**
  * @brief FDCAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_FDCAN1_Init(void)
{

  /* USER CODE BEGIN FDCAN1_Init 0 */

  /* USER CODE END FDCAN1_Init 0 */

  /* USER CODE BEGIN FDCAN1_Init 1 */

  /* USER CODE END FDCAN1_Init 1 */
  hfdcan1.Instance = FDCAN1;
  hfdcan1.Init.ClockDivider = FDCAN_CLOCK_DIV1;
  hfdcan1.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
  hfdcan1.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan1.Init.AutoRetransmission = DISABLE;
  hfdcan1.Init.TransmitPause = DISABLE;
  hfdcan1.Init.ProtocolException = DISABLE;
  hfdcan1.Init.NominalPrescaler = 10;
  hfdcan1.Init.NominalSyncJumpWidth = 2;
  hfdcan1.Init.NominalTimeSeg1 = 13;
  hfdcan1.Init.NominalTimeSeg2 = 2;
  hfdcan1.Init.DataPrescaler = 1;
  hfdcan1.Init.DataSyncJumpWidth = 2;
  hfdcan1.Init.DataTimeSeg1 = 1;
  hfdcan1.Init.DataTimeSeg2 = 1;
  hfdcan1.Init.StdFiltersNbr = 0;
  hfdcan1.Init.ExtFiltersNbr = 0;
  hfdcan1.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FDCAN1_Init 2 */

  /* USER CODE END FDCAN1_Init 2 */

}

/**
  * @brief FDCAN2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_FDCAN2_Init(void)
{

  /* USER CODE BEGIN FDCAN2_Init 0 */

  /* USER CODE END FDCAN2_Init 0 */

  /* USER CODE BEGIN FDCAN2_Init 1 */

  /* USER CODE END FDCAN2_Init 1 */
  hfdcan2.Instance = FDCAN2;
  hfdcan2.Init.ClockDivider = FDCAN_CLOCK_DIV1;
  hfdcan2.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
  hfdcan2.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan2.Init.AutoRetransmission = DISABLE;
  hfdcan2.Init.TransmitPause = DISABLE;
  hfdcan2.Init.ProtocolException = DISABLE;
  hfdcan2.Init.NominalPrescaler = 10;
  hfdcan2.Init.NominalSyncJumpWidth = 2;
  hfdcan2.Init.NominalTimeSeg1 = 13;
  hfdcan2.Init.NominalTimeSeg2 = 2;
  hfdcan2.Init.DataPrescaler = 1;
  hfdcan2.Init.DataSyncJumpWidth = 1;
  hfdcan2.Init.DataTimeSeg1 = 1;
  hfdcan2.Init.DataTimeSeg2 = 1;
  hfdcan2.Init.StdFiltersNbr = 0;
  hfdcan2.Init.ExtFiltersNbr = 0;
  hfdcan2.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  if (HAL_FDCAN_Init(&hfdcan2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FDCAN2_Init 2 */

  /* USER CODE END FDCAN2_Init 2 */

}

/**
  * @brief IWDG Initialization Function
  * @param None
  * @retval None
  */
static void MX_IWDG_Init(void)
{

  /* USER CODE BEGIN IWDG_Init 0 */

  /* USER CODE END IWDG_Init 0 */

  /* USER CODE BEGIN IWDG_Init 1 */

  /* USER CODE END IWDG_Init 1 */
  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_16;
  hiwdg.Init.Window = 12000;
  hiwdg.Init.Reload = 5000;
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN IWDG_Init 2 */

  /* USER CODE END IWDG_Init 2 */

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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 63;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 99;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMAMUX1_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA1_Channel2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);

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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LED4_Pin|LED3_Pin|SAFETY_Pin|BOOOT_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, LED2_Pin|LED1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LED4_Pin LED3_Pin SAFETY_Pin */
  GPIO_InitStruct.Pin = LED4_Pin|LED3_Pin|SAFETY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : LED2_Pin LED1_Pin */
  GPIO_InitStruct.Pin = LED2_Pin|LED1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : Sense_Inertia_Pin */
  GPIO_InitStruct.Pin = Sense_Inertia_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Sense_Inertia_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : Sense_EBS_Pin Sense_Left_Pin Sense_Driver_Pin */
  GPIO_InitStruct.Pin = Sense_EBS_Pin|Sense_Left_Pin|Sense_Driver_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : Sense_Left_Wheel_Pin */
  GPIO_InitStruct.Pin = Sense_Left_Wheel_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(Sense_Left_Wheel_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : Sense_Right_Wheel_Pin Sense_Overtravel_Pin Sense_Right_Kill_Pin Sense_BSPD_Pin */
  GPIO_InitStruct.Pin = Sense_Right_Wheel_Pin|Sense_Overtravel_Pin|Sense_Right_Kill_Pin|Sense_BSPD_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : BOOOT_Pin */
  GPIO_InitStruct.Pin = BOOOT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(BOOOT_GPIO_Port, &GPIO_InitStruct);

  /**/
  __HAL_SYSCFG_FASTMODEPLUS_ENABLE(SYSCFG_FASTMODEPLUS_PB8);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartMainTask */
/**
  * @brief  Function implementing the MainTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartMainTask */
void StartMainTask(void *argument)
{
  /* USER CODE BEGIN 5 */
	/* Set APPS reference voltage */
	HAL_DAC_Start(&hdac1, DAC_CHANNEL_2);
	HAL_DAC_Start(&hdac2, DAC_CHANNEL_1);
	HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_2, DAC_ALIGN_12B_R, 1950); // ~2200  mV 2733
	HAL_DAC_SetValue(&hdac2, DAC_CHANNEL_1, DAC_ALIGN_12B_R, 2100); // ~2418 mV 3003

	HAL_GPIO_WritePin(BOOOT_GPIO_Port, BOOOT_Pin, GPIO_PIN_RESET);
	// APPS
	HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
	HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED);

	HAL_ADC_Start_DMA(&hadc1, reinterpret_cast<uint32_t*>(adc1_dma_buffer), 150);
	HAL_ADC_Start_DMA(&hadc2, reinterpret_cast<uint32_t*>(adc2_dma_buffer), 250);
	HAL_TIM_Base_Start(&htim2);

	HAL_FDCAN_Start(&hfdcan1);
	HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);

	HAL_FDCAN_Start(&hfdcan2);
	HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);

	HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, GPIO_PIN_SET);
	/* Infinite loop */
	for(;;)
	{
		apps_value_to_send = apps.get_value_to_send();
		brakePressureValueToSend = brakes.get_raw_avg_press_value();
		steering_position_to_send = analogs.get_steering_position();

		PUTM_CAN::DriverInput drvInput = {
			  .pedalPosition = apps_value_to_send,
			  .brakePressureFront = brakePressureValueToSend.first,
			  .brakePressureRear = brakePressureValueToSend.second,
			  .steeringWheelPosition = (int16_t)steering_position_to_send
		};

		auto driverInputFrame = PUTM_CAN::Can_tx_message<PUTM_CAN::DriverInput>(drvInput, PUTM_CAN::can_tx_header_DRIVER_INPUT);
		HAL_StatusTypeDef status = driverInputFrame.send(hfdcan1);
		UNUSED(status);

		sc_state = sc.update_val();
		PUTM_CAN::FrontData frontData = {
	  	 			  .sense_left_kill    = static_cast<bool>(sc_state & 0x01),
	  	 			  .sense_right_kill   = static_cast<bool>(sc_state & 0x02),
	  	 			  .sense_driver_kill  = static_cast<bool>(sc_state & 0x03),
	  	 			  .sense_inertia      = static_cast<bool>(sc_state & 0x04),
	  	 			  .sense_bspd         = static_cast<bool>(sc_state & 0x05),
	  	 			  .sense_overtravel   = static_cast<bool>(sc_state & 0x06),
	  	 			  .sense_right_wheel  = true
		};
		if (brakePressureValueToSend.first > brakes.FRONT_BRAKING_THRESHOLD || brakePressureValueToSend.second > brakes.REAR_BRAKING_THRESHOLD)
		{
			frontData.is_braking = true;
		}
	  	auto frontDataFrame =  PUTM_CAN::Can_tx_message<PUTM_CAN::FrontData>(frontData, PUTM_CAN::can_tx_header_FRONT_DATA);
	  	status = frontDataFrame.send(hfdcan1);
	  	HAL_IWDG_Refresh(&hiwdg);
	  	osDelay(25);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartBlinkTask */
PUTM_CAN::PcMainData pcMain;
/**
* @brief Function implementing the BlinkTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartBlinkTask */
void StartBlinkTask(void *argument)
{
  /* USER CODE BEGIN StartBlinkTask */
  /* Infinite loop */
	osDelay(500);
	for(;;)
	{
		PUTM_CAN::Dashboard dsh;
//		PUTM_CAN::PcMainData pcMain;

		if (PUTM_CAN::can.get_dashboard_new_data())
		{
			auto dash_ts_button = PUTM_CAN::can.get_dashboard().ts_activation_button;
			auto dash_rtd_button = PUTM_CAN::can.get_dashboard().ready_to_drive_button;

			/* Check if we want to enable TS voltage */
			if (dash_ts_button == true)
			{
				dsh.ts_activation_button = 1;
				HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);
			}

			/* Act according to current rtd state */
//			switch(rtd){
//			case true:
//				if (dash_rtd_button == true)
//				{
//					/* Escape rtd */
//					rtd = false;
//				}
//				break;
//
//			case false:
//				/* If NOT in rtd, check if we want to enter it */
//				if (dash_rtd_button == true and brakePressureValueToSend.first >= brakes.FRONT_BRAKING_THRESHOLD and brakePressureValueToSend.second >= brakes.REAR_BRAKING_THRESHOLD)
//				{
//					/* Enter rtd */
//					rtd = true;
//					HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);
//				}
//				break;
//			}
//			pcMain.rtd = rtd;
		}
		osDelay(50);
		HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET);
		HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);

//		auto amkRearLeftData = PUTM_CAN::can.get_amk_rear_left_actual_values2();
//		auto amkRearRightData = PUTM_CAN::can.get_amk_rear_right_actual_values2();
//
//		auto amkRearLeftMain = PUTM_CAN::can.get_amk_rear_left_actual_values1();
//		auto amkRearRightMain = PUTM_CAN::can.get_amk_rear_right_actual_values1();
//
//		auto tsVoltage = PUTM_CAN::can.get_bms_hv_main().voltage_sum;
//
//		pcMain.invertersReady = inverterStatus;
//		pcMain.rearLeftInverterTemperature = amkRearLeftData.AMK_TempIGBT / 10;
//		pcMain.rearRightInverterTemperature = amkRearRightData.AMK_TempIGBT / 10;
//		pcMain.rearLeftMotorTemperature = amkRearLeftData.AMK_TempMotor / 10;
//		pcMain.rearRightMotorTemperature = amkRearRightData.AMK_TempMotor / 10;
//		pcMain.rtd = rtd;
//		pcMain.power = ((amkRearLeftMain.AMK_TorqueCurrent * 107.20) / 16384) + ((amkRearRightMain.AMK_TorqueCurrent * 107.20) / 16384) * tsVoltage;
//		float rpm = (amkRearLeftMain.AMK_ActualVelocity + amkRearRightMain.AMK_ActualVelocity)/2.0;
//		pcMain.vehicleSpeed = ((amkRearLeftMain.AMK_ActualVelocity + amkRearRightMain.AMK_ActualVelocity)/2) * 60 * 2 * 3.14 * (405.0/2.0) * 0.0000001;
//		pcMain.rpm = (uint32_t)rpm;

//		auto pc_main = PUTM_CAN::Can_tx_message<PUTM_CAN::PcMainData>(pcMain, PUTM_CAN::can_tx_header_PC_MAIN_DATA);
//		auto status = pc_main.send(hfdcan1);
//		UNUSED(status);

		auto dash = PUTM_CAN::Can_tx_message<PUTM_CAN::Dashboard>(dsh, PUTM_CAN::can_tx_header_DASHBOARD);
		auto status = dash.send(hfdcan1);
		UNUSED(status);
	}
  /* USER CODE END StartBlinkTask */
}

/* USER CODE BEGIN Header_StartAmkTask */

/**
* @brief Function implementing the AmkTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartAmkTask */
void StartAmkTask(void *argument)
{
  /* USER CODE BEGIN StartAmkTask */
  /* Infinite loop */

	PUTM_CAN::AmkFrontLeftSetpoints1  frontLeftAmkSetpoints = {};
	PUTM_CAN::AmkFrontRightSetpoints1 frontRightAmkSetpoints = {};
	PUTM_CAN::AmkRearLeftSetpoints1   rearLeftAmkSetpoints = {};
	PUTM_CAN::AmkRearRightSetpoints1  rearRightAmkSetpoints = {};

	uint32_t timeout = 0;
	constexpr uint32_t amk_startup_timeout = 5000;

	PUTM_CAN::PcMainData pcMain = {
		  .rtd = rtd,
	};

	for(;;)
	{
		auto frontLeftAmk  = PUTM_CAN::can.get_amk_front_left_actual_values1();
		auto frontRightAmk = PUTM_CAN::can.get_amk_front_right_actual_values1();
		auto rearLeftAmk   = PUTM_CAN::can.get_amk_rear_left_actual_values1();
		auto rearRightAmk  = PUTM_CAN::can.get_amk_rear_right_actual_values1();

	switch(state)
	{
		case StateMachine::UNDEFINED:
		{
		  if (frontLeftAmk.AMK_Status.AMK_bError || frontRightAmk.AMK_Status.AMK_bError || rearLeftAmk.AMK_Status.AMK_bError || rearRightAmk.AMK_Status.AMK_bError )
		  {
	        state = StateMachine::ERROR_RESET;
	      }
	      if (frontLeftAmk.AMK_Status.AMK_bSystemReady && frontRightAmk.AMK_Status.AMK_bSystemReady && rearLeftAmk.AMK_Status.AMK_bSystemReady && rearRightAmk.AMK_Status.AMK_bSystemReady)
	      {
	    	  state = StateMachine::IDLING;
	      }
		}
	    break;
		case StateMachine::ERROR_RESET:
		{
		      if (frontLeftAmk.AMK_Status.AMK_bSystemReady && frontRightAmk.AMK_Status.AMK_bSystemReady && rearLeftAmk.AMK_Status.AMK_bSystemReady && rearRightAmk.AMK_Status.AMK_bSystemReady)
		      {
		    	frontLeftAmkSetpoints.AMK_Control.AMK_bErrorReset = false;
		    	frontRightAmkSetpoints.AMK_Control.AMK_bErrorReset = false;
		    	rearLeftAmkSetpoints.AMK_Control.AMK_bErrorReset = false;
		    	rearRightAmkSetpoints.AMK_Control.AMK_bErrorReset = false;
		        state = StateMachine::IDLING;
		      }
		      else
		      {
		    	frontLeftAmkSetpoints.AMK_Control.AMK_bErrorReset = true;
		    	frontRightAmkSetpoints.AMK_Control.AMK_bErrorReset = true;
			    rearLeftAmkSetpoints.AMK_Control.AMK_bErrorReset = true;
			    rearRightAmkSetpoints.AMK_Control.AMK_bErrorReset = true;
		      }
		}
		break;
		case StateMachine::IDLING:
		{
			inverterStatus = false;
			if ((frontLeftAmk.AMK_Status.AMK_bError or frontRightAmk.AMK_Status.AMK_bError or rearLeftAmk.AMK_Status.AMK_bError  or rearRightAmk.AMK_Status.AMK_bError))
			{
//				state = StateMachine::ERROR_RESET;
			}
			if (rtd == true)
			{
				state = StateMachine::STARTUP;
				timeout = xTaskGetTickCount();
			}
		}
		break;

		case StateMachine::STARTUP:
		{
			 if ((xTaskGetTickCount() - timeout) > amk_startup_timeout)
			 {
				 /* Stop startup and go to idle through error reset */
				 state = StateMachine::SWITCH_OFF;
				 rtd = false;
				 timeout = 0;
			 }
			 if ((frontLeftAmk.AMK_Status.AMK_bError or frontRightAmk.AMK_Status.AMK_bError or rearLeftAmk.AMK_Status.AMK_bError or rearRightAmk.AMK_Status.AMK_bError))
			 {
				 state = StateMachine::ERROR_RESET;
			 }

			 frontLeftAmkSetpoints.AMK_Control.AMK_bDcOn = true;
			 frontRightAmkSetpoints.AMK_Control.AMK_bDcOn = true;
			 rearLeftAmkSetpoints.AMK_Control.AMK_bDcOn = true;
			 rearRightAmkSetpoints.AMK_Control.AMK_bDcOn = true;

			 frontLeftAmkSetpoints.AMK_TorqueLimitNegativ  = 0;
			 frontRightAmkSetpoints.AMK_TorqueLimitNegativ = 0;
			 rearLeftAmkSetpoints.AMK_TorqueLimitNegativ   = 0;
			 rearRightAmkSetpoints.AMK_TorqueLimitNegativ  = 0;

			 frontLeftAmkSetpoints.AMK_TorqueLimitPositiv  = 0;
			 frontRightAmkSetpoints.AMK_TorqueLimitPositiv = 0;
			 rearLeftAmkSetpoints.AMK_TorqueLimitPositiv   = 0;
			 rearRightAmkSetpoints.AMK_TorqueLimitPositiv  = 0;

			 frontLeftAmkSetpoints.AMK_TargetVelocity  = 0;
			 frontRightAmkSetpoints.AMK_TargetVelocity = 0;
			 rearLeftAmkSetpoints.AMK_TargetVelocity   = 0;
			 rearRightAmkSetpoints.AMK_TargetVelocity  = 0;


			 if (!frontLeftAmk.AMK_Status.AMK_bDcOn && !frontRightAmk.AMK_Status.AMK_bDcOn && !rearLeftAmk.AMK_Status.AMK_bDcOn && !rearRightAmk.AMK_Status.AMK_bDcOn)
			 {
			     break;
			 }

			 frontLeftAmkSetpoints.AMK_Control.AMK_bInverterOn = true;
			 frontRightAmkSetpoints.AMK_Control.AMK_bInverterOn = true;
			 rearLeftAmkSetpoints.AMK_Control.AMK_bInverterOn = true;
			 rearRightAmkSetpoints.AMK_Control.AMK_bInverterOn = true;

			 frontLeftAmkSetpoints.AMK_Control.AMK_bEnable = true;
			 frontRightAmkSetpoints.AMK_Control.AMK_bEnable = true;
			 rearLeftAmkSetpoints.AMK_Control.AMK_bEnable = true;
			 rearRightAmkSetpoints.AMK_Control.AMK_bEnable = true;

			 if (!frontLeftAmk.AMK_Status.AMK_bInverterOn && !frontRightAmk.AMK_Status.AMK_bInverterOn  && !rearLeftAmk.AMK_Status.AMK_bInverterOn  && !rearRightAmk.AMK_Status.AMK_bInverterOn )
			 {
			     break;
			 }
			 if (!(frontLeftAmk.AMK_Status.AMK_bQuitInverterOn && frontRightAmk.AMK_Status.AMK_bQuitInverterOn && rearLeftAmk.AMK_Status.AMK_bQuitInverterOn  && rearRightAmk.AMK_Status.AMK_bQuitInverterOn))
			 {
			     break;
			 }
			 else
			 {
				 /* Lastly, check if driver is not pressing acceleration pedal */
				 if (apps_value_to_send > 0)
				 {
					 break;
				 }
				 else
				 {
					 state = StateMachine::TORQUE_CONTROL;
					 HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_RESET);
				 }
			 }
		}
		break;

		case StateMachine::TORQUE_CONTROL:
		{
			/* Check some stop conditions*/
			if ((frontLeftAmk.AMK_Status.AMK_bError or frontRightAmk.AMK_Status.AMK_bError or rearLeftAmk.AMK_Status.AMK_bError or rearRightAmk.AMK_Status.AMK_bError))
			{
				state = StateMachine::SWITCH_OFF;
				rtd = false;
				break;
			}

			inverterStatus = true;

			frontLeftAmkSetpoints.AMK_TorqueLimitNegativ  = -1000;
			frontRightAmkSetpoints.AMK_TorqueLimitNegativ = -1000;
			rearLeftAmkSetpoints.AMK_TorqueLimitNegativ   = -1000;
			rearRightAmkSetpoints.AMK_TorqueLimitNegativ  = -1000;

			frontLeftAmkSetpoints.AMK_TorqueLimitPositiv  = 1000;
			frontRightAmkSetpoints.AMK_TorqueLimitPositiv = 1000;
			rearLeftAmkSetpoints.AMK_TorqueLimitPositiv   = 1000;
			rearRightAmkSetpoints.AMK_TorqueLimitPositiv  = 1000;

			float target_torque = (apps_value_to_send / 500.0) * 1000;

//			if (brakePressureValueToSend.first > 500 and frontLeftAmk.AMK_ActualVelocity > 0)
//			{
//				target_torque = -20.f;
//			}

			frontLeftAmkSetpoints.AMK_TargetVelocity  =  -1.0 * target_torque * 0.7;
			frontRightAmkSetpoints.AMK_TargetVelocity = target_torque * 0.7;
			rearLeftAmkSetpoints.AMK_TargetVelocity   = target_torque;
			rearRightAmkSetpoints.AMK_TargetVelocity  = -1.0  * target_torque;

			if (!rtd)
			{
				state = StateMachine::SWITCH_OFF;
				osDelay(10);
				break;
			}
		}
		break;

		case StateMachine::SWITCH_OFF:
		{

			frontLeftAmkSetpoints.AMK_Control.AMK_bInverterOn = false;
			frontRightAmkSetpoints.AMK_Control.AMK_bInverterOn = false;
			rearLeftAmkSetpoints.AMK_Control.AMK_bInverterOn = false;
			rearRightAmkSetpoints.AMK_Control.AMK_bInverterOn = false;

			frontLeftAmkSetpoints.AMK_Control.AMK_bEnable = false;
			frontRightAmkSetpoints.AMK_Control.AMK_bEnable = false;
			rearLeftAmkSetpoints.AMK_Control.AMK_bEnable = false;
			rearRightAmkSetpoints.AMK_Control.AMK_bEnable = false;

			frontLeftAmkSetpoints.AMK_Control.AMK_bDcOn = false;
			frontRightAmkSetpoints.AMK_Control.AMK_bDcOn = false;
			rearLeftAmkSetpoints.AMK_Control.AMK_bDcOn = false;
			rearRightAmkSetpoints.AMK_Control.AMK_bDcOn = false;

			if ((frontLeftAmk.AMK_Status.AMK_bError or frontRightAmk.AMK_Status.AMK_bError or rearLeftAmk.AMK_Status.AMK_bError  or rearRightAmk.AMK_Status.AMK_bError))
			{
//				state = StateMachine::ERROR_RESET;
			}

			/* Wait until inverter 0 is switched-off.*/
			if (frontLeftAmk.AMK_Status.AMK_bInverterOn || frontRightAmk.AMK_Status.AMK_bInverterOn || rearLeftAmk.AMK_Status.AMK_bInverterOn || rearRightAmk.AMK_Status.AMK_bInverterOn)
			{
				break;
			}
			state = StateMachine::IDLING;
		}
		break;
		default:
		{

		}
		break;
	}

	auto frontLeftSetpoint  = PUTM_CAN::Can_tx_message<PUTM_CAN::AmkFrontLeftSetpoints1  > (frontLeftAmkSetpoints,  PUTM_CAN::can_tx_header_AMK_FRONT_LEFT_SETPOINTS);
	auto frontRightSetpoint = PUTM_CAN::Can_tx_message<PUTM_CAN::AmkFrontRightSetpoints1 > (frontRightAmkSetpoints, PUTM_CAN::can_tx_header_AMK_FRONT_RIGHT_SETPOINTS);
	auto rearLefttSetpoint  = PUTM_CAN::Can_tx_message<PUTM_CAN::AmkRearLeftSetpoints1   > (rearLeftAmkSetpoints,   PUTM_CAN::can_tx_header_AMK_REAR_LEFT_SETPOINTS);
	auto rearRightSetpoint  = PUTM_CAN::Can_tx_message<PUTM_CAN::AmkRearRightSetpoints1  > (rearRightAmkSetpoints,  PUTM_CAN::can_tx_header_AMK_REAR_RIGHT_SETPOINTS);


	osDelay(1);
	frontLeftSetpoint.send(hfdcan2);
	osDelay(1);
	frontRightSetpoint.send(hfdcan2);
	osDelay(1);
	rearLefttSetpoint.send(hfdcan2);
	osDelay(1);
	rearRightSetpoint.send(hfdcan2);

	osDelay(25);
  }
  /* USER CODE END StartAmkTask */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
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

#ifdef  USE_FULL_ASSERT
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

/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "stm32g4xx_hal.h"

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

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define SAFETY_CONTROL_Pin GPIO_PIN_13
#define SAFETY_CONTROL_GPIO_Port GPIOC
#define SAFETY_BSPD_Pin GPIO_PIN_14
#define SAFETY_BSPD_GPIO_Port GPIOC
#define SAFETY_OVERTRAVEL_Pin GPIO_PIN_15
#define SAFETY_OVERTRAVEL_GPIO_Port GPIOC
#define SAFETY_INERTIA_Pin GPIO_PIN_0
#define SAFETY_INERTIA_GPIO_Port GPIOC
#define SAFETY_DRIVER_Pin GPIO_PIN_1
#define SAFETY_DRIVER_GPIO_Port GPIOC
#define SAFETY_RIGHT_Pin GPIO_PIN_2
#define SAFETY_RIGHT_GPIO_Port GPIOC
#define SAFETY_LEFT_Pin GPIO_PIN_3
#define SAFETY_LEFT_GPIO_Port GPIOC
#define ADC1_APPS2_Pin GPIO_PIN_1
#define ADC1_APPS2_GPIO_Port GPIOA
#define ADC1_APPS1_Pin GPIO_PIN_2
#define ADC1_APPS1_GPIO_Port GPIOA
#define ADC1_EXTRA_1_Pin GPIO_PIN_3
#define ADC1_EXTRA_1_GPIO_Port GPIOA
#define ADC2_EXTRA_2_Pin GPIO_PIN_4
#define ADC2_EXTRA_2_GPIO_Port GPIOA
#define ADC2_SUSP_R_Pin GPIO_PIN_6
#define ADC2_SUSP_R_GPIO_Port GPIOA
#define ADC2_SUSP_L_Pin GPIO_PIN_7
#define ADC2_SUSP_L_GPIO_Port GPIOA
#define ADC2_BRAKE_PRESSURE_REAR_Pin GPIO_PIN_4
#define ADC2_BRAKE_PRESSURE_REAR_GPIO_Port GPIOC
#define ADC2_BRAKE_PRESSURE_FRONT_Pin GPIO_PIN_5
#define ADC2_BRAKE_PRESSURE_FRONT_GPIO_Port GPIOC
#define ADC1_BRAKE_PPS_Pin GPIO_PIN_0
#define ADC1_BRAKE_PPS_GPIO_Port GPIOB
#define TIM5_LAPTIMER_Pin GPIO_PIN_2
#define TIM5_LAPTIMER_GPIO_Port GPIOB
#define DEBUG_5_Pin GPIO_PIN_6
#define DEBUG_5_GPIO_Port GPIOC
#define DEBUG_4_Pin GPIO_PIN_7
#define DEBUG_4_GPIO_Port GPIOC
#define DEBUG_3_Pin GPIO_PIN_8
#define DEBUG_3_GPIO_Port GPIOC
#define DEBUG_2_Pin GPIO_PIN_9
#define DEBUG_2_GPIO_Port GPIOC
#define DEBUG_1_Pin GPIO_PIN_8
#define DEBUG_1_GPIO_Port GPIOA
#define TIM3_SWPS_DATA_Pin GPIO_PIN_4
#define TIM3_SWPS_DATA_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

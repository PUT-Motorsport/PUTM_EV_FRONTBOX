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
#define SENSE_OVERTRAVEL_Pin GPIO_PIN_13
#define SENSE_OVERTRAVEL_GPIO_Port GPIOC
#define SENSE_EBS_Pin GPIO_PIN_15
#define SENSE_EBS_GPIO_Port GPIOC
#define SENSE_LEFT_Pin GPIO_PIN_0
#define SENSE_LEFT_GPIO_Port GPIOC
#define SENSE_RIGHT_Pin GPIO_PIN_1
#define SENSE_RIGHT_GPIO_Port GPIOC
#define SENSE_DRIVER_Pin GPIO_PIN_2
#define SENSE_DRIVER_GPIO_Port GPIOC
#define SENSE_INERTIA_Pin GPIO_PIN_3
#define SENSE_INERTIA_GPIO_Port GPIOC
#define SAFETY_CONTROL_Pin GPIO_PIN_0
#define SAFETY_CONTROL_GPIO_Port GPIOA
#define APPS2_ADC1_Pin GPIO_PIN_3
#define APPS2_ADC1_GPIO_Port GPIOA
#define APPS1_ADC2_Pin GPIO_PIN_4
#define APPS1_ADC2_GPIO_Port GPIOA
#define SUSP_L_ADC2_Pin GPIO_PIN_6
#define SUSP_L_ADC2_GPIO_Port GPIOA
#define SUSP_R_ADC2_Pin GPIO_PIN_7
#define SUSP_R_ADC2_GPIO_Port GPIOA
#define ADC2_BRAKE_PRESSURE_FRONT_Pin GPIO_PIN_4
#define ADC2_BRAKE_PRESSURE_FRONT_GPIO_Port GPIOC
#define ADC2_BRAKE_PRESSURE_REAR_Pin GPIO_PIN_5
#define ADC2_BRAKE_PRESSURE_REAR_GPIO_Port GPIOC
#define DEBUG4_Pin GPIO_PIN_14
#define DEBUG4_GPIO_Port GPIOB
#define DEBUG3_Pin GPIO_PIN_15
#define DEBUG3_GPIO_Port GPIOB
#define DEBUG2_Pin GPIO_PIN_6
#define DEBUG2_GPIO_Port GPIOC
#define DEBUG1_Pin GPIO_PIN_7
#define DEBUG1_GPIO_Port GPIOC
#define DEBUG5_Pin GPIO_PIN_9
#define DEBUG5_GPIO_Port GPIOA
#define SENSE_BSPD_Pin GPIO_PIN_9
#define SENSE_BSPD_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

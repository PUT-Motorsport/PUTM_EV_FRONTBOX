/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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
#define LED4_Pin GPIO_PIN_14
#define LED4_GPIO_Port GPIOB
#define LED3_Pin GPIO_PIN_15
#define LED3_GPIO_Port GPIOB
#define LED2_Pin GPIO_PIN_6
#define LED2_GPIO_Port GPIOC
#define LED1_Pin GPIO_PIN_7
#define LED1_GPIO_Port GPIOC
#define Sense_EBS_Pin GPIO_PIN_10
#define Sense_EBS_GPIO_Port GPIOC
#define Sense_Left_Pin GPIO_PIN_11
#define Sense_Left_GPIO_Port GPIOC
#define Sense_Driver_Pin GPIO_PIN_12
#define Sense_Driver_GPIO_Port GPIOC
#define SAFETY_Pin GPIO_PIN_3
#define SAFETY_GPIO_Port GPIOB
#define Sense_Left_Wheel_Pin GPIO_PIN_4
#define Sense_Left_Wheel_GPIO_Port GPIOB
#define Sense_Right_Wheel_Pin GPIO_PIN_6
#define Sense_Right_Wheel_GPIO_Port GPIOB
#define Sense_Overtravel_Pin GPIO_PIN_7
#define Sense_Overtravel_GPIO_Port GPIOB
#define Sense_Right_Pin GPIO_PIN_8
#define Sense_Right_GPIO_Port GPIOB
#define Sense_BSPD_Pin GPIO_PIN_9
#define Sense_BSPD_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

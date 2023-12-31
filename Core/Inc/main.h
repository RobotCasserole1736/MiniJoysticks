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
#include "stm32f4xx_hal.h"

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
#define LED_3_Pin GPIO_PIN_15
#define LED_3_GPIO_Port GPIOA
#define LED_2_Pin GPIO_PIN_10
#define LED_2_GPIO_Port GPIOA
#define LED_0_Pin GPIO_PIN_8
#define LED_0_GPIO_Port GPIOA
#define LED_1_Pin GPIO_PIN_9
#define LED_1_GPIO_Port GPIOA
#define JOY_BTN_Pin GPIO_PIN_3
#define JOY_BTN_GPIO_Port GPIOA
#define JOY_Z_Pin GPIO_PIN_2
#define JOY_Z_GPIO_Port GPIOA
#define BTN_3_Pin GPIO_PIN_7
#define BTN_3_GPIO_Port GPIOA
#define BTN_2_Pin GPIO_PIN_6
#define BTN_2_GPIO_Port GPIOA
#define BTN_1_Pin GPIO_PIN_5
#define BTN_1_GPIO_Port GPIOA
#define JOY_Y_Pin GPIO_PIN_0
#define JOY_Y_GPIO_Port GPIOA
#define BTN_0_Pin GPIO_PIN_4
#define BTN_0_GPIO_Port GPIOA
#define JOY_X_Pin GPIO_PIN_1
#define JOY_X_GPIO_Port GPIOA
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

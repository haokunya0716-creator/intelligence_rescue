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
#define E1B_Pin GPIO_PIN_12
#define E1B_GPIO_Port GPIOB
#define E1B_EXTI_IRQn EXTI15_10_IRQn
#define E1A_Pin GPIO_PIN_13
#define E1A_GPIO_Port GPIOB
#define E1A_EXTI_IRQn EXTI15_10_IRQn
#define E2B_Pin GPIO_PIN_14
#define E2B_GPIO_Port GPIOB
#define E2B_EXTI_IRQn EXTI15_10_IRQn
#define E2A_Pin GPIO_PIN_15
#define E2A_GPIO_Port GPIOB
#define E2A_EXTI_IRQn EXTI15_10_IRQn
#define PH_1_IN_Pin GPIO_PIN_9
#define PH_1_IN_GPIO_Port GPIOD
#define PH_2_IN_Pin GPIO_PIN_10
#define PH_2_IN_GPIO_Port GPIOD
#define KEY3_Pin GPIO_PIN_9
#define KEY3_GPIO_Port GPIOC
#define KEY2_Pin GPIO_PIN_11
#define KEY2_GPIO_Port GPIOC

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

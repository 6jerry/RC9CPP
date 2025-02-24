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
<<<<<<< HEAD
#define shoot_key_Pin GPIO_PIN_7
#define shoot_key_GPIO_Port GPIOF
#define Right_Wall_Pin GPIO_PIN_0
#define Right_Wall_GPIO_Port GPIOA
#define Action_Pin GPIO_PIN_1
#define Action_GPIO_Port GPIOA
#define Red_Pin GPIO_PIN_7
#define Red_GPIO_Port GPIOA
#define Left_Hand_Pin GPIO_PIN_15
#define Left_Hand_GPIO_Port GPIOB
#define Right__Hand_Pin GPIO_PIN_14
#define Right__Hand_GPIO_Port GPIOD
#define Left_Wall_Pin GPIO_PIN_9
#define Left_Wall_GPIO_Port GPIOC
#define Blue_Pin GPIO_PIN_4
#define Blue_GPIO_Port GPIOB
=======
>>>>>>> origin/main

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

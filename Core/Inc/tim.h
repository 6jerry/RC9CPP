/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    tim.h
  * @brief   This file contains all the function prototypes for
  *          the tim.c file
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
#ifndef __TIM_H__
#define __TIM_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

<<<<<<< HEAD
<<<<<<< HEAD
extern TIM_HandleTypeDef htim2;

=======
extern TIM_HandleTypeDef htim1;

extern TIM_HandleTypeDef htim2;

=======
extern TIM_HandleTypeDef htim1;

extern TIM_HandleTypeDef htim2;

>>>>>>> e9e92ea34931924eedf6897a5078319b30f9357d
extern TIM_HandleTypeDef htim3;

extern TIM_HandleTypeDef htim4;

extern TIM_HandleTypeDef htim8;

extern TIM_HandleTypeDef htim9;

extern TIM_HandleTypeDef htim12;

<<<<<<< HEAD
>>>>>>> origin/main
=======
>>>>>>> e9e92ea34931924eedf6897a5078319b30f9357d
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

<<<<<<< HEAD
<<<<<<< HEAD
void MX_TIM2_Init(void);
=======
void MX_TIM1_Init(void);
void MX_TIM2_Init(void);
=======
void MX_TIM1_Init(void);
void MX_TIM2_Init(void);
>>>>>>> e9e92ea34931924eedf6897a5078319b30f9357d
void MX_TIM3_Init(void);
void MX_TIM4_Init(void);
void MX_TIM8_Init(void);
void MX_TIM9_Init(void);
void MX_TIM12_Init(void);

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
<<<<<<< HEAD
>>>>>>> origin/main
=======
>>>>>>> e9e92ea34931924eedf6897a5078319b30f9357d

/* USER CODE BEGIN Prototypes */

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __TIM_H__ */


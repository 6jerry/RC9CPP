/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f4xx_hal_timebase_tim.c
  * @brief   HAL time base based on the hardware TIM.
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
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_tim.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
<<<<<<< HEAD
<<<<<<< HEAD
TIM_HandleTypeDef        htim3;
=======
TIM_HandleTypeDef        htim5;
>>>>>>> origin/main
=======
TIM_HandleTypeDef        htim5;
>>>>>>> e9e92ea34931924eedf6897a5078319b30f9357d
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/**
<<<<<<< HEAD
<<<<<<< HEAD
  * @brief  This function configures the TIM3 as a time base source.
=======
  * @brief  This function configures the TIM5 as a time base source.
>>>>>>> origin/main
=======
  * @brief  This function configures the TIM5 as a time base source.
>>>>>>> e9e92ea34931924eedf6897a5078319b30f9357d
  *         The time source is configured  to have 1ms time base with a dedicated
  *         Tick interrupt priority.
  * @note   This function is called  automatically at the beginning of program after
  *         reset by HAL_Init() or at any time when clock is configured, by HAL_RCC_ClockConfig().
  * @param  TickPriority: Tick interrupt priority.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_InitTick(uint32_t TickPriority)
{
  RCC_ClkInitTypeDef    clkconfig;
  uint32_t              uwTimclock, uwAPB1Prescaler = 0U;

  uint32_t              uwPrescalerValue = 0U;
  uint32_t              pFLatency;
  HAL_StatusTypeDef     status;

<<<<<<< HEAD
<<<<<<< HEAD
  /* Enable TIM3 clock */
  __HAL_RCC_TIM3_CLK_ENABLE();
=======
  /* Enable TIM5 clock */
  __HAL_RCC_TIM5_CLK_ENABLE();
>>>>>>> origin/main
=======
  /* Enable TIM5 clock */
  __HAL_RCC_TIM5_CLK_ENABLE();
>>>>>>> e9e92ea34931924eedf6897a5078319b30f9357d

  /* Get clock configuration */
  HAL_RCC_GetClockConfig(&clkconfig, &pFLatency);

  /* Get APB1 prescaler */
  uwAPB1Prescaler = clkconfig.APB1CLKDivider;
<<<<<<< HEAD
<<<<<<< HEAD
  /* Compute TIM3 clock */
=======
  /* Compute TIM5 clock */
>>>>>>> origin/main
=======
  /* Compute TIM5 clock */
>>>>>>> e9e92ea34931924eedf6897a5078319b30f9357d
  if (uwAPB1Prescaler == RCC_HCLK_DIV1)
  {
    uwTimclock = HAL_RCC_GetPCLK1Freq();
  }
  else
  {
    uwTimclock = 2UL * HAL_RCC_GetPCLK1Freq();
  }

<<<<<<< HEAD
<<<<<<< HEAD
  /* Compute the prescaler value to have TIM3 counter clock equal to 1MHz */
=======
  /* Compute the prescaler value to have TIM5 counter clock equal to 1MHz */
>>>>>>> e9e92ea34931924eedf6897a5078319b30f9357d
  uwPrescalerValue = (uint32_t) ((uwTimclock / 1000000U) - 1U);

  /* Initialize TIM5 */
  htim5.Instance = TIM5;

  /* Initialize TIMx peripheral as follow:

<<<<<<< HEAD
  + Period = [(TIM3CLK/1000) - 1]. to have a (1/1000) s time base.
=======
  /* Compute the prescaler value to have TIM5 counter clock equal to 1MHz */
  uwPrescalerValue = (uint32_t) ((uwTimclock / 1000000U) - 1U);

  /* Initialize TIM5 */
  htim5.Instance = TIM5;

  /* Initialize TIMx peripheral as follow:

  + Period = [(TIM5CLK/1000) - 1]. to have a (1/1000) s time base.
>>>>>>> origin/main
=======
  + Period = [(TIM5CLK/1000) - 1]. to have a (1/1000) s time base.
>>>>>>> e9e92ea34931924eedf6897a5078319b30f9357d
  + Prescaler = (uwTimclock/1000000 - 1) to have a 1MHz counter clock.
  + ClockDivision = 0
  + Counter direction = Up
  */
<<<<<<< HEAD
<<<<<<< HEAD
  htim3.Init.Period = (1000000U / 1000U) - 1U;
  htim3.Init.Prescaler = uwPrescalerValue;
  htim3.Init.ClockDivision = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
=======
  htim5.Init.Period = (1000000U / 1000U) - 1U;
  htim5.Init.Prescaler = uwPrescalerValue;
  htim5.Init.ClockDivision = 0;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
>>>>>>> e9e92ea34931924eedf6897a5078319b30f9357d

  status = HAL_TIM_Base_Init(&htim5);
  if (status == HAL_OK)
  {
    /* Start the TIM time Base generation in interrupt mode */
    status = HAL_TIM_Base_Start_IT(&htim5);
    if (status == HAL_OK)
    {
<<<<<<< HEAD
    /* Enable the TIM3 global Interrupt */
        HAL_NVIC_EnableIRQ(TIM3_IRQn);
=======
  htim5.Init.Period = (1000000U / 1000U) - 1U;
  htim5.Init.Prescaler = uwPrescalerValue;
  htim5.Init.ClockDivision = 0;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

  status = HAL_TIM_Base_Init(&htim5);
  if (status == HAL_OK)
  {
    /* Start the TIM time Base generation in interrupt mode */
    status = HAL_TIM_Base_Start_IT(&htim5);
    if (status == HAL_OK)
    {
    /* Enable the TIM5 global Interrupt */
        HAL_NVIC_EnableIRQ(TIM5_IRQn);
>>>>>>> origin/main
=======
    /* Enable the TIM5 global Interrupt */
        HAL_NVIC_EnableIRQ(TIM5_IRQn);
>>>>>>> e9e92ea34931924eedf6897a5078319b30f9357d
      /* Configure the SysTick IRQ priority */
      if (TickPriority < (1UL << __NVIC_PRIO_BITS))
      {
        /* Configure the TIM IRQ priority */
<<<<<<< HEAD
<<<<<<< HEAD
        HAL_NVIC_SetPriority(TIM3_IRQn, TickPriority, 0U);
=======
        HAL_NVIC_SetPriority(TIM5_IRQn, TickPriority, 0U);
>>>>>>> origin/main
=======
        HAL_NVIC_SetPriority(TIM5_IRQn, TickPriority, 0U);
>>>>>>> e9e92ea34931924eedf6897a5078319b30f9357d
        uwTickPrio = TickPriority;
      }
      else
      {
        status = HAL_ERROR;
      }
    }
  }

 /* Return function status */
  return status;
}

/**
  * @brief  Suspend Tick increment.
<<<<<<< HEAD
<<<<<<< HEAD
  * @note   Disable the tick increment by disabling TIM3 update interrupt.
=======
  * @note   Disable the tick increment by disabling TIM5 update interrupt.
>>>>>>> origin/main
=======
  * @note   Disable the tick increment by disabling TIM5 update interrupt.
>>>>>>> e9e92ea34931924eedf6897a5078319b30f9357d
  * @param  None
  * @retval None
  */
void HAL_SuspendTick(void)
{
<<<<<<< HEAD
<<<<<<< HEAD
  /* Disable TIM3 update Interrupt */
  __HAL_TIM_DISABLE_IT(&htim3, TIM_IT_UPDATE);
=======
  /* Disable TIM5 update Interrupt */
  __HAL_TIM_DISABLE_IT(&htim5, TIM_IT_UPDATE);
>>>>>>> origin/main
=======
  /* Disable TIM5 update Interrupt */
  __HAL_TIM_DISABLE_IT(&htim5, TIM_IT_UPDATE);
>>>>>>> e9e92ea34931924eedf6897a5078319b30f9357d
}

/**
  * @brief  Resume Tick increment.
<<<<<<< HEAD
<<<<<<< HEAD
  * @note   Enable the tick increment by Enabling TIM3 update interrupt.
=======
  * @note   Enable the tick increment by Enabling TIM5 update interrupt.
>>>>>>> origin/main
=======
  * @note   Enable the tick increment by Enabling TIM5 update interrupt.
>>>>>>> e9e92ea34931924eedf6897a5078319b30f9357d
  * @param  None
  * @retval None
  */
void HAL_ResumeTick(void)
{
<<<<<<< HEAD
<<<<<<< HEAD
  /* Enable TIM3 Update interrupt */
  __HAL_TIM_ENABLE_IT(&htim3, TIM_IT_UPDATE);
=======
  /* Enable TIM5 Update interrupt */
  __HAL_TIM_ENABLE_IT(&htim5, TIM_IT_UPDATE);
>>>>>>> origin/main
=======
  /* Enable TIM5 Update interrupt */
  __HAL_TIM_ENABLE_IT(&htim5, TIM_IT_UPDATE);
>>>>>>> e9e92ea34931924eedf6897a5078319b30f9357d
}


/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file    can.c
 * @brief   This file provides code for the configuration
 *          of the CAN instances.
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
#include "can.h"

/* USER CODE BEGIN 0 */

#include <stm32f4xx_hal_can.h>
#include <stm32f407xx.h>
CAN_TxHeaderTypeDef TxHeader; // 发�??
uint8_t RxData[8];            // 数据接收数组，can的数据帧只有8�???
uint8_t RxData2[8];
int Rx_Flag = 1;
/* USER CODE END 0 */

CAN_HandleTypeDef hcan1;
CAN_HandleTypeDef hcan2;

/* CAN1 init function */
void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 3;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_9TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_4TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */
  // CAN1_Filter_Init();
  // HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING); // �???活指定的中断函数

  /* USER CODE END CAN1_Init 2 */

}
/* CAN2 init function */
void MX_CAN2_Init(void)
{

  /* USER CODE BEGIN CAN2_Init 0 */

  /* USER CODE END CAN2_Init 0 */

  /* USER CODE BEGIN CAN2_Init 1 */

  /* USER CODE END CAN2_Init 1 */
  hcan2.Instance = CAN2;
  hcan2.Init.Prescaler = 3;
  hcan2.Init.Mode = CAN_MODE_NORMAL;
  hcan2.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan2.Init.TimeSeg1 = CAN_BS1_9TQ;
  hcan2.Init.TimeSeg2 = CAN_BS2_4TQ;
  hcan2.Init.TimeTriggeredMode = DISABLE;
  hcan2.Init.AutoBusOff = ENABLE;
  hcan2.Init.AutoWakeUp = DISABLE;
  hcan2.Init.AutoRetransmission = DISABLE;
  hcan2.Init.ReceiveFifoLocked = DISABLE;
  hcan2.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN2_Init 2 */
  // CAN2_Filter_Init();
  // HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);
  /* USER CODE END CAN2_Init 2 */

}

static uint32_t HAL_RCC_CAN1_CLK_ENABLED=0;

void HAL_CAN_MspInit(CAN_HandleTypeDef* canHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(canHandle->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspInit 0 */

  /* USER CODE END CAN1_MspInit 0 */
    /* CAN1 clock enable */
    HAL_RCC_CAN1_CLK_ENABLED++;
    if(HAL_RCC_CAN1_CLK_ENABLED==1){
      __HAL_RCC_CAN1_CLK_ENABLE();
    }

    __HAL_RCC_GPIOD_CLK_ENABLE();
    /**CAN1 GPIO Configuration
    PD0     ------> CAN1_RX
    PD1     ------> CAN1_TX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF9_CAN1;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

    /* CAN1 interrupt Init */
    HAL_NVIC_SetPriority(CAN1_RX0_IRQn, 7, 0);
    HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);
  /* USER CODE BEGIN CAN1_MspInit 1 */

  /* USER CODE END CAN1_MspInit 1 */
  }
  else if(canHandle->Instance==CAN2)
  {
  /* USER CODE BEGIN CAN2_MspInit 0 */

  /* USER CODE END CAN2_MspInit 0 */
    /* CAN2 clock enable */
    __HAL_RCC_CAN2_CLK_ENABLE();
    HAL_RCC_CAN1_CLK_ENABLED++;
    if(HAL_RCC_CAN1_CLK_ENABLED==1){
      __HAL_RCC_CAN1_CLK_ENABLE();
    }

    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**CAN2 GPIO Configuration
    PB12     ------> CAN2_RX
    PB13     ------> CAN2_TX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF9_CAN2;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* CAN2 interrupt Init */
    HAL_NVIC_SetPriority(CAN2_RX0_IRQn, 8, 0);
    HAL_NVIC_EnableIRQ(CAN2_RX0_IRQn);
  /* USER CODE BEGIN CAN2_MspInit 1 */

  /* USER CODE END CAN2_MspInit 1 */
  }
}

void HAL_CAN_MspDeInit(CAN_HandleTypeDef* canHandle)
{

  if(canHandle->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspDeInit 0 */

  /* USER CODE END CAN1_MspDeInit 0 */
    /* Peripheral clock disable */
    HAL_RCC_CAN1_CLK_ENABLED--;
    if(HAL_RCC_CAN1_CLK_ENABLED==0){
      __HAL_RCC_CAN1_CLK_DISABLE();
    }

    /**CAN1 GPIO Configuration
    PD0     ------> CAN1_RX
    PD1     ------> CAN1_TX
    */
    HAL_GPIO_DeInit(GPIOD, GPIO_PIN_0|GPIO_PIN_1);

    /* CAN1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(CAN1_RX0_IRQn);
  /* USER CODE BEGIN CAN1_MspDeInit 1 */

  /* USER CODE END CAN1_MspDeInit 1 */
  }
  else if(canHandle->Instance==CAN2)
  {
  /* USER CODE BEGIN CAN2_MspDeInit 0 */

  /* USER CODE END CAN2_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_CAN2_CLK_DISABLE();
    HAL_RCC_CAN1_CLK_ENABLED--;
    if(HAL_RCC_CAN1_CLK_ENABLED==0){
      __HAL_RCC_CAN1_CLK_DISABLE();
    }

    /**CAN2 GPIO Configuration
    PB12     ------> CAN2_RX
    PB13     ------> CAN2_TX
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_12|GPIO_PIN_13);

    /* CAN2 interrupt Deinit */
    HAL_NVIC_DisableIRQ(CAN2_RX0_IRQn);
  /* USER CODE BEGIN CAN2_MspDeInit 1 */

  /* USER CODE END CAN2_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */
/*CAN过滤器初始化*/
void CAN1_Filter_Init(void)
{
  CAN_FilterTypeDef sFilterConfig;

  sFilterConfig.FilterBank = 0;                      /* 过滤器组0 */
  sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;  /* 屏蔽位模�??? */
  sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT; /* 32位�??*/

  sFilterConfig.FilterIdHigh = (((uint32_t)CAN_RxExtId << 3) & 0xFFFF0000) >> 16; /* 要过滤的ID高位 */                  // 0x0000
  sFilterConfig.FilterIdLow = (((uint32_t)CAN_RxExtId << 3) | CAN_ID_EXT | CAN_RTR_DATA) & 0xFFFF; /* 要过滤的ID低位 */ // 0x0000
  //  sFilterConfig.FilterMaskIdHigh     = 0xFFFF;			/* 过滤器高16位每位必须匹�??? */
  //  sFilterConfig.FilterMaskIdLow      = 0xFFFF;			/* 过滤器低16位每位必须匹�??? */
  sFilterConfig.FilterMaskIdHigh = 0x0000;           /* 实际上是关闭了过滤器 */
  sFilterConfig.FilterMaskIdLow = 0x0000;            /* 实际上是关闭了过滤器 */
  sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0; /* 过滤器被关联到FIFO 0 */
  sFilterConfig.FilterActivation = ENABLE;           /* 使能过滤�??? */
  // sFilterConfig.SlaveStartFilterBank = 14;

  if (HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig) != HAL_OK)
  {
    /* Filter configuration Error */
    Error_Handler();
  }

  if (HAL_CAN_Start(&hcan1) != HAL_OK)
  {
    /* Start Error */
    Error_Handler();
  }

  /*##-4- Activate CAN RX notification #######################################*/
  if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
  {
    /* Start Error */
    Error_Handler();
  }

  TxHeader.ExtId = CAN_TxExtId; // 扩展标识�???(29�???)
  TxHeader.IDE = CAN_ID_EXT;    // 使用标准�???
  TxHeader.RTR = CAN_RTR_DATA;  // 数据�???
  TxHeader.DLC = 8;
  TxHeader.TransmitGlobalTime = DISABLE;
}

/*CAN过滤器初始化*/
void CAN2_Filter_Init(void)
{
  CAN_FilterTypeDef sFilterConfig;

  sFilterConfig.FilterBank = 14;                     /* 过滤器组0 */
  sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;  /* 屏蔽位模�??? */
  sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT; /* 32位�??*/

  sFilterConfig.FilterIdHigh = (((uint32_t)CAN_RxExtId << 3) & 0xFFFF0000) >> 16; /* 要过滤的ID高位 */                  // 0x0000
  sFilterConfig.FilterIdLow = (((uint32_t)CAN_RxExtId << 3) | CAN_ID_EXT | CAN_RTR_DATA) & 0xFFFF; /* 要过滤的ID低位 */ // 0x0000
  //  sFilterConfig.FilterMaskIdHigh     = 0xFFFF;			/* 过滤器高16位每位必须匹�??? */
  //  sFilterConfig.FilterMaskIdLow      = 0xFFFF;			/* 过滤器低16位每位必须匹�??? */
  sFilterConfig.FilterMaskIdHigh = 0x0000;
  sFilterConfig.FilterMaskIdLow = 0x0000;
  sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0; /* 过滤器被关联到FIFO 0 */
  sFilterConfig.FilterActivation = ENABLE;           /* 使能过滤�??? */
  sFilterConfig.SlaveStartFilterBank = 14;

  if (HAL_CAN_ConfigFilter(&hcan2, &sFilterConfig) != HAL_OK)
  {
    /* Filter configuration Error */
    Error_Handler();
  }

  if (HAL_CAN_Start(&hcan2) != HAL_OK)
  {
    /* Start Error */
    Error_Handler();
  }

  /*##-4- Activate CAN RX notification #######################################*/
  if (HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
  {
    /* Start Error */
    Error_Handler();
  }

  TxHeader.ExtId = CAN_TxExtId; // 扩展标识�???(29�???)
  TxHeader.IDE = CAN_ID_EXT;    // 使用标准�???
  TxHeader.RTR = CAN_RTR_DATA;  // 数据�???
  TxHeader.DLC = 8;
  TxHeader.TransmitGlobalTime = DISABLE;
}

/*CAN接收中断函数*/
/*void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{

  if (hcan == &hcan1)
  {
    CAN_RxHeaderTypeDef RxHeader; // 接收
    Rx_Flag = 1;                  // 接收标志�???
    HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &RxHeader, RxData);
    // m3508_update_info(&RxHeader, RxData); // M3508电机数据处理
  }
  if (hcan == &hcan2)
  {
    CAN_RxHeaderTypeDef RxHeader2; // 接收
    Rx_Flag = 0;                   // 接收标志�???
    HAL_CAN_GetRxMessage(&hcan2, CAN_RX_FIFO0, &RxHeader2, RxData2);
    // shoot_motor_update(&RxHeader2, RxData2);
  }
}*/
/*void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  if (hcan == &hcan2)
  {
    CAN_RxHeaderTypeDef RxHeader; // 接收
    Rx_Flag = 0;                  // 接收标志�???
    HAL_CAN_GetRxMessage(&hcan2, CAN_RX_FIFO1, &RxHeader, RxData2);
  }
}
/* USER CODE END 1 */

/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    can.h
  * @brief   This file contains all the function prototypes for
  *          the can.c file
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
#ifndef __CAN_H__
#define __CAN_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

extern CAN_HandleTypeDef hcan1;

extern CAN_HandleTypeDef hcan2;

/* USER CODE BEGIN Private defines */
#define CAN_RxExtId 0x0000
#define CAN_TxExtId 0x0000
/* USER CODE END Private defines */

void MX_CAN1_Init(void);
void MX_CAN2_Init(void);

/* USER CODE BEGIN Prototypes */
<<<<<<< HEAD
<<<<<<< HEAD
void CAN1_Filter_Init(void);   //¹ýÂËÆ÷ÅäÖÃº¯Êý
void CAN2_Filter_Init(void);   //¹ýÂËÆ÷ÅäÖÃº¯Êý
uint8_t CAN_Send_Msg(uint8_t* msg,uint8_t len);  //Êý¾Ý·¢ËÍº¯Êý
 
extern CAN_TxHeaderTypeDef	TxHeader;      //·¢ËÍ
extern CAN_RxHeaderTypeDef	RxHeader;      //½ÓÊÕ
extern uint8_t	RxData[8];   //Êý¾Ý½ÓÊÕÊý×é
=======
void CAN1_Filter_Init(void);   //ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ãºï¿½ï¿½ï¿½
void CAN2_Filter_Init(void);   //ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ãºï¿½ï¿½ï¿½
uint8_t CAN_Send_Msg(uint8_t* msg,uint8_t len);  //ï¿½ï¿½ï¿½Ý·ï¿½ï¿½Íºï¿½ï¿½ï¿½
 
extern CAN_TxHeaderTypeDef	TxHeader;      //ï¿½ï¿½ï¿½ï¿½
extern CAN_RxHeaderTypeDef	RxHeader;      //ï¿½ï¿½ï¿½ï¿½
extern uint8_t	RxData[8];   //ï¿½ï¿½ï¿½Ý½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½
>>>>>>> origin/main
=======
void CAN1_Filter_Init(void);   //ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ãºï¿½ï¿½ï¿½
void CAN2_Filter_Init(void);   //ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ãºï¿½ï¿½ï¿½
uint8_t CAN_Send_Msg(uint8_t* msg,uint8_t len);  //ï¿½ï¿½ï¿½Ý·ï¿½ï¿½Íºï¿½ï¿½ï¿½
 
extern CAN_TxHeaderTypeDef	TxHeader;      //ï¿½ï¿½ï¿½ï¿½
extern CAN_RxHeaderTypeDef	RxHeader;      //ï¿½ï¿½ï¿½ï¿½
extern uint8_t	RxData[8];   //ï¿½ï¿½ï¿½Ý½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½
>>>>>>> e9e92ea34931924eedf6897a5078319b30f9357d

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __CAN_H__ */


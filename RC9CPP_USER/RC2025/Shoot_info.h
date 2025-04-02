//
// Created by 15828 on 2025/4/2.
//

#ifndef RC9CPP_SHOOT_INFO_H
#define RC9CPP_SHOOT_INFO_H

#ifdef __cplusplus
extern "C"
{
#endif
#include "motor.h"
#include "TaskManager.h"
#include "encoder.h"
#include <arm_math.h>
#include "wit_gyro.h"
#include <math.h>
#ifdef __cplusplus
}
#endif
#ifdef __cplusplus



class Shoot_info : public ITaskProcessor, public Encoder, public wit_gyro
{
private:
    float max_shooter_speed = 600.0f;

    float Current_distance=0,Current_Angle=0;
    float K;
    float Target_distance;

public:
    float Get_Current_distance();
    float Get_Current_Angle();
    float Set_Target_distance(float Target_distance_);

    Shoot_info(UART_HandleTypeDef *Encoder_huart_,UART_HandleTypeDef *Wit_huart_) : Encoder(Encoder_huart_),wit_gyro(Wit_huart_){}

    float Speed_Cal();

    void process_data();
};
#endif
#endif

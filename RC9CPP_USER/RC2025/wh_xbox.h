//
// Created by 15828 on 2025/3/20.
//

#ifndef RC9CPP_WH_XBOX_H
#define RC9CPP_WH_XBOX_H

#ifdef __cplusplus
extern "C"
{
#endif
#include "xbox.h"
#include "motor.h"
#include "TaskManager.h"
#include "wh_setup.h"
#include "encoder.h"
#include <arm_math.h>
#ifdef __cplusplus
}
#endif
#ifdef __cplusplus

class wh_xbox : public xbox, public ITaskProcessor
{
private:

    GPIO_TypeDef *Catcher_port = nullptr, *Shooter_port = nullptr,*lock_port = nullptr;
    uint16_t Catcher_pin = 0, Shooter_pin = 0,lock_pin = 0;

    uint8_t Stop_flag=0,Catcher_flag=0,Speed_level=0,Shoot_flag=0,lock_flag=0;
    float Speed_map;

    float Target_distance = 0;
    float Now_distance = 0;
    Encoder *encoder = nullptr;

    float max_lifter_speed = 420.0f,
    max_turn_speed = 80.0f,
    max_shooter_speed = 600.0f,
    max_pithcer_speed = 430.0f;

    power_motor *lifter_motor=nullptr, *turn_motor=nullptr, *shooter_motor=nullptr, *pithcer_motor=nullptr;
public:
    void process_data();
    void xbox_init();
    void btn_scan();
    void load_motor(power_motor *lifter_motor_, power_motor *turn_motor_, power_motor *shooter_motor_, power_motor *pithcer_motor_);
    void load_pin(GPIO_TypeDef *Catcher_port_,uint16_t Catcher_pin_,GPIO_TypeDef *lock_port_,uint16_t lock_pin_,GPIO_TypeDef *Shooter_port_,uint16_t Shooter_pin_);
    void load_encoder(Encoder *encoder_);
};

#endif
#endif //RC9CPP_WH_XBOX_H

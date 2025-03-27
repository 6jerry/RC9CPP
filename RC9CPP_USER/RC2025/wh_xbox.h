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
#include "chassis.h"
#include "wh_setup.h"

#include <arm_math.h>
#ifdef __cplusplus
}
#endif
#ifdef __cplusplus

class wh_xbox : public xbox, public ITaskProcessor
{
private:

    GPIO_TypeDef *Catcher_port = nullptr, *Shooter_port = nullptr;
    uint16_t Catcher_pin = 0, Shooter_pin = 0;

    uint8_t Stop_flag=0,Catcher_flag=0,Speed_level=0,Shoot_flag=0;

    power_motor *lifter_motor=nullptr, *turn_motor=nullptr, *shooter_motor=nullptr, *pithcer_motor=nullptr;
    GPIO_TypeDef *trigger_port=nullptr, *shooter_port = nullptr;
public:
    void process_data();
    void xbox_init();
    void btn_scan();
    void load_motor(power_motor *lifter_motor_, power_motor *turn_motor_, power_motor *shooter_motor_, power_motor *pithcer_motor_);
    void load_pin(GPIO_TypeDef *Catcher_port_,GPIO_TypeDef *Shooter_port_,uint16_t Catcher_pin_,uint16_t Shooter_pin_);
};

#endif
#endif //RC9CPP_WH_XBOX_H

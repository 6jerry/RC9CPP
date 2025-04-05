#ifndef Up_XBOX_H
#define Up_XBOX_H

#ifdef __cplusplus
extern "C"
{
#endif
#include "xbox.h"
#include "TaskManager.h"
#include "motor.h"
#include "gpio.h"
#include "TrapezoidalPlanner.h"
#include "robot_chassis.h"
#include "Vector2D.h"
#ifdef __cplusplus
}
#endif
#ifdef __cplusplus

class UserCtrl_xbox : public ITaskProcessor, public xbox
{
private:
    power_motor *lifting_motor = nullptr, *turn_motor = nullptr, *shooter_motor = nullptr, *pitching_motor = nullptr;

    GPIO_TypeDef *clamp_port = nullptr, *shooter_port = nullptr,*brakes_port = nullptr;
    uint16_t clamp_pin = 0, shooter_pin = 0, brakes_pin = 0;

    uint8_t if_motor_start = 0, clamp_flag = 0, shoot_flag = 0, brakes_flag = 0;

    float max_lifting_speed = 352.0f;
    float max_turn_speed = 300.0f;
    float max_shooter_speed = 400.0f;
    float max_pitching_speed = 300.0f;
    float Speed_map = 0.8f; // 速度映射比例
    uint8_t Speed_level = 1; // 速度等级

public:
    void process_data();
    UserCtrl_xbox();
    void btn_scan();
    void btnconfig_init();
    void add_motor(power_motor *lifting_motor_, power_motor *turn_motor_, power_motor *shooter_motor_, power_motor *pitching_motor_);
    void add_trigger(GPIO_TypeDef *clamp_port_, uint16_t clamp_pin_, GPIO_TypeDef *shooter_port_, uint16_t shooter_pin_, GPIO_TypeDef *brakes_port_, uint16_t brakes_pin_);
};

#endif
#endif
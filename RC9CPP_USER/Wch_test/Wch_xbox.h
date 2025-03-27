#ifndef RC_TEST_XBOX_H
#define RC_TEST_XBOX_H

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

class UserCtrl_xbox : public ITaskProcessor, public xbox, public chassis_user
{
private:
    power_motor *lifting_motor = nullptr, *turn_motor = nullptr, *shooter_motor = nullptr, *pitching_motor = nullptr;

    GPIO_TypeDef *clamp_port = nullptr, *shooter_port = nullptr;
    uint16_t clamp_pin = 0, shooter_pin = 0;

    uint8_t if_motor_start = 0, clamp_flag = 0, shoot_flag = 0, lock_flag = 0;

    float max_lifting_speed = 3520.0f;
    float max_turn_speed = 80.0f;
    float max_shooter_speed = 4000.0f;
    float max_pitching_speed = 3000.0f;

    VelocityPlanner x_planner, y_planner, w_planner;
    float full_speed = 0.0f, full_w = 0.0f;
    uint8_t priocode = 1;

public:
    void process_data();
    UserCtrl_xbox(float full_speed_, float full_w_);
    void btn_scan();
    void btnconfig_init();
    void add_motor(power_motor *lifting_motor_, power_motor *turn_motor_, power_motor *shooter_motor_, power_motor *pitching_motor_);
    void add_trigger(GPIO_TypeDef *clamp_port_, uint16_t clamp_pin_, GPIO_TypeDef *shooter_port_, uint16_t shooter_pin_);
    void init_plan(float max_xy_acc, float max_w_acc);
};

#endif
#endif
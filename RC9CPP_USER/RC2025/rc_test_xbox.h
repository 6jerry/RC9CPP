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
#ifdef __cplusplus
}
#endif
#ifdef __cplusplus

class yun_ball_xbox : public ITaskProcessor, public xbox
{
private:
    power_motor *lfter_motor = nullptr, *turn_motor = nullptr, *shooter_motor = nullptr, *pithcer_motor = nullptr;

    GPIO_TypeDef *trigger_port = nullptr, *shooter_port = nullptr;
    uint16_t trigger_pin = 0, shooter_pin = 0;

    uint8_t if_motor_start = 0, trigger_start = 0, shooter_trigger = 0, shoot_or_yun = 0;

    float max_lifter_speed = 420.0f, max_turn_speed = 80.0f;

public:
    void process_data();
    yun_ball_xbox();
    void btn_scan();
    void btnconfig_init();
    void add_motor(power_motor *lfter_motor_, power_motor *turn_motor_, power_motor *shooter_motor_, power_motor *pithcer_motor_);
    void add_trigger(GPIO_TypeDef *trigger_port_, uint16_t trigger_pin_, GPIO_TypeDef *shooter_port_, uint16_t shooter_pin_);
};

#endif
#endif
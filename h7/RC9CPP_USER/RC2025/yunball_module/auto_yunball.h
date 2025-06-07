#ifndef AUTO_YUNBALL_H
#define AUTO_YUNBALL_H

#ifdef __cplusplus
extern "C"
{
#endif
#include "TaskManager.h"
#include "SuperPID.h"
#include "motor.h"
#include "PID.h"
#include "SuperPID.h"
#include "gpio.h"
#ifdef __cplusplus
}
#endif

#ifdef __cplusplus

enum state_flag
{
    static_flag,
    yunball_flag,
    putball_flag,
    stop_flag
};

class auto_yunball : public ITaskProcessor
{
private:
    GPIO_TypeDef *lift_port, *claw_port, *push_port;
    uint16_t lift_pin, claw_pin, push_pin;
    float get_speed = 0.0f;
    float max_turn_speed = 70.0f;

    void set_claw(bool if_open);
    void set_lift(bool if_up);
    void set_push(bool if_push);
    void yunball();
    void putball();
    power_motor *turn_motor;
    state_flag flag = static_flag;

public:
    uint8_t mode_flag = 2, start_flag = 0, lb_flag = 0, rb_flag = 0, cnt_flag = 0, up_flag = 0, down_flag = 0, left_flag = 0, right_flag = 0;

    void process_data();
    void add_motor(power_motor *turn_motor_);
    void add_io(GPIO_TypeDef *lift_port_, uint16_t lift_pin_,  GPIO_TypeDef *claw_port_, uint16_t claw_pin_, GPIO_TypeDef *push_port_, uint16_t push_pin_);

    void control_motor(float speed_);       //外部控制接口
    void control_claw(bool if_open);
    void control_lift(bool if_up);
    void control_push(bool if_push);

    void start_yunball();
    void start_putball();
    void stop();
};


#endif
#endif
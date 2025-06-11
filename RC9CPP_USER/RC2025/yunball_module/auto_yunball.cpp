#include "auto_yunball.h"

void auto_yunball::process_data()
{
    switch(flag)
    {
        case static_flag:
            turn_motor->set_rpm(-get_speed * max_turn_speed);
            break;
        case yunball_flag:
            yunball();
            break;
        case putball_flag:
            putball();
            break;
        case stop_flag:
            osDelay(200);
            flag = static_flag;
            break;
        default:
            break;
    }
}

void auto_yunball::add_io(GPIO_TypeDef *lift_port_, uint16_t lift_pin_,  GPIO_TypeDef *claw_port_, uint16_t claw_pin_, GPIO_TypeDef *push_port_, uint16_t push_pin_)
{
    lift_port = lift_port_;
    lift_pin = lift_pin_;
    claw_port = claw_port_;
    claw_pin = claw_pin_;
    push_port = push_port_;
    push_pin = push_pin_;
}

void auto_yunball::add_motor(power_motor *turn_motor_)
{
    turn_motor = turn_motor_;
}

//外部接口函数
bool auto_yunball::start_yunball()
{
    if(flag == static_flag) {flag = yunball_flag;}
    if(flag == stop_flag) {return true;}
    return false;
}
bool auto_yunball::start_putball()
{
    if(flag == static_flag) {flag = putball_flag;}
    if(flag == stop_flag) {return true;}
    return false;
}

void auto_yunball::control_motor(float speed_){get_speed = speed_;}
void auto_yunball::control_claw(bool if_open){if(flag == static_flag)set_claw(if_open);}
void auto_yunball::control_push(bool if_push){if(flag == static_flag)set_push(if_push);}
void auto_yunball::control_lift(bool if_up){if(flag== static_flag)set_lift(if_up);}



//内部实现函数
void auto_yunball::yunball()
{
    set_claw(true);
    set_push(true);
    osDelay(200); 
    set_push(false);
    osDelay(300);
    set_claw(false);
    flag = stop_flag;
}
void auto_yunball::putball()
{
    set_lift(true);
    osDelay(700);
    turn_motor->set_pos_speedplan(-180.0f, 20.0f, 10.0f, 10.0f, 0.0f);
    osDelay(1700);
    set_claw(true);
    osDelay(1000);
    shooter->set_shooter_mode(shooter_lift);
    osDelay(800);
    turn_motor->set_pos_speedplan(-90.0f, 20.0f, 10.0f, 10.0f, 0.0f);
    osDelay(800);
    set_lift(false);
    osDelay(500);
    shooter->set_shooter_mode(shooter_stop);
    flag = stop_flag;
}

void auto_yunball::add_shooter(AutoShooter *shooter_)
{
    shooter = shooter_;
}


void auto_yunball::set_claw(bool if_open) {HAL_GPIO_WritePin(claw_port, claw_pin, if_open ? GPIO_PIN_RESET : GPIO_PIN_SET);}
void auto_yunball::set_lift(bool if_up) {HAL_GPIO_WritePin(lift_port, lift_pin, if_up ? GPIO_PIN_SET : GPIO_PIN_RESET);}
void auto_yunball::set_push(bool if_push) {HAL_GPIO_WritePin(push_port, push_pin, if_push ? GPIO_PIN_SET : GPIO_PIN_RESET);}
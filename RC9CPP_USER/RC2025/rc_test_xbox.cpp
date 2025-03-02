#include "rc_test_xbox.h"

void yun_ball_xbox::btnconfig_init()
{
    btnAConfig = {
        &xbox_msgs.btnA,
        &xbox_msgs.btnA_last,
        &if_motor_start,
        1,
        ButtonActionType::Toggle,
        nullptr};

    btnRBConfig = {
        &xbox_msgs.btnRB,
        &xbox_msgs.btnRB_last,
        &trigger_start,
        1,
        ButtonActionType::Toggle,
        nullptr};
}

void yun_ball_xbox::btn_scan()
{
    handleButton(btnAConfig);
    handleButton(btnRBConfig);
}

yun_ball_xbox::yun_ball_xbox()
{
    btnconfig_init();
}

void yun_ball_xbox::process_data()
{
    btn_scan();
    joymap_compute();

    if (trigger_start == 1)
    {
        HAL_GPIO_WritePin(trigger_port, trigger_pin, GPIO_PIN_SET);
    }
    else if (trigger_start == 0)
    {
        HAL_GPIO_WritePin(trigger_port, trigger_pin, GPIO_PIN_RESET);
    }

    if (if_motor_start == 1)
    {
        lfter_motor->set_rpm(xbox_msgs.joyLVert_map * max_lifter_speed);
        turn_motor->set_rpm(xbox_msgs.joyRHori_map * max_turn_speed);
    }
    else if (if_motor_start == 0)
    {
        lfter_motor->set_rpm(0.0f);
        turn_motor->set_rpm(0.0f);
    }
}
void yun_ball_xbox::add_motor(power_motor *lfter_motor_, power_motor *turn_motor_)
{
    lfter_motor = lfter_motor_;
    turn_motor = turn_motor_;
}

void yun_ball_xbox::add_trigger(GPIO_TypeDef *trigger_port_, uint16_t trigger_pin_)
{
    trigger_port = trigger_port_;
    trigger_pin = trigger_pin_;
}

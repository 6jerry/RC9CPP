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

    btnYConfig = {
        &xbox_msgs.btnY,
        &xbox_msgs.btnY_last,
        &shoot_or_yun,
        1,
        ButtonActionType::Toggle,
        nullptr};

    btnLBConfig = {
        &xbox_msgs.btnLB,
        &xbox_msgs.btnLB_last,
        &shooter_trigger,
        1,
        ButtonActionType::Toggle,
        nullptr};
}

void yun_ball_xbox::btn_scan()
{
    handleButton(btnAConfig);
    handleButton(btnRBConfig);
    handleButton(btnYConfig);
    handleButton(btnLBConfig);
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

    if (shooter_trigger == 1)
    {
        HAL_GPIO_WritePin(shooter_port, shooter_pin, GPIO_PIN_SET);
    }
    else if (shooter_trigger == 0)
    {
        HAL_GPIO_WritePin(shooter_port, shooter_pin, GPIO_PIN_RESET);
    }

    if (if_motor_start == 1)
    {
        if (shoot_or_yun == 0)
        {
            lfter_motor->set_rpm(xbox_msgs.joyLVert_map * max_lifter_speed);
            turn_motor->set_rpm(xbox_msgs.joyRHori_map * max_turn_speed);
        }
        else if (shoot_or_yun == 1)
        {
            shooter_motor->set_rpm(-xbox_msgs.joyLVert_map * max_lifter_speed);
            pithcer_motor->set_rpm(-xbox_msgs.joyRVert_map * max_turn_speed);
        }
    }
    else if (if_motor_start == 0)
    {
        lfter_motor->set_rpm(0.0f);
        turn_motor->set_rpm(0.0f);
        shooter_motor->set_rpm(0.0f);
        pithcer_motor->set_rpm(0.0f);
    }
}
void yun_ball_xbox::add_motor(power_motor *lfter_motor_, power_motor *turn_motor_, power_motor *shooter_motor_, power_motor *pithcer_motor_)
{
    lfter_motor = lfter_motor_;
    turn_motor = turn_motor_;
    shooter_motor = shooter_motor_;
    pithcer_motor = pithcer_motor_;
}

void yun_ball_xbox::add_trigger(GPIO_TypeDef *trigger_port_, uint16_t trigger_pin_, GPIO_TypeDef *shooter_port_, uint16_t shooter_pin_)
{
    trigger_port = trigger_port_;
    trigger_pin = trigger_pin_;
    shooter_port = shooter_port_;
    shooter_pin = shooter_pin_;
}

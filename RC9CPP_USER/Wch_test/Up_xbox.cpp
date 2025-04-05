#include "Wch_xbox.h"

void UserCtrl_xbox::btnconfig_init()
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
        &shoot_flag,
        1,
        ButtonActionType::Toggle,
        nullptr};

    btnLBConfig = {
        &xbox_msgs.btnLB,
        &xbox_msgs.btnLB_last,
        &brakes_flag,
        1,
        ButtonActionType::Toggle,
        nullptr};

    btnRSConfig = {
        &xbox_msgs.btnRS,
        &xbox_msgs.btnRS_last,
        &clamp_flag,
        1,
        ButtonActionType::Toggle,
        nullptr};

    btnXConfig = {
        &xbox_msgs.btnX,
        &xbox_msgs.btnX_last,
        &speed_level,
        2,
        ButtonActionType::Decrement,
        nullptr};
    btnBConfig = {
        &xbox_msgs.btnB,
        &xbox_msgs.btnB_last,
        &speed_level,
        2,
        ButtonActionType::Increment,
        nullptr};
}

void UserCtrl_xbox::btn_scan()
{
    handleButton(btnAConfig);
    handleButton(btnRBConfig);
    handleButton(btnLBConfig);
    handleButton(btnRSConfig);
    handleButton(btnXConfig);
    handleButton(btnBConfig);
}

UserCtrl_xbox::UserCtrl_xbox()
{
    btnconfig_init();
}

void UserCtrl_xbox::process_data()
{
    btn_scan();
    joymap_compute();

    if (clamp_flag == 1)
    {
        HAL_GPIO_WritePin(clamp_port, clamp_pin, GPIO_PIN_SET);
    }
    else if (clamp_flag == 0)
    {
        HAL_GPIO_WritePin(clamp_port, clamp_pin, GPIO_PIN_RESET);
    }

    if (shoot_flag == 1)
    {
        HAL_GPIO_WritePin(shooter_port, shooter_pin, GPIO_PIN_SET);
    }
    else if (shoot_flag == 0)
    {
        HAL_GPIO_WritePin(shooter_port, shooter_pin, GPIO_PIN_RESET);
    }

    if (brakes_flag == 1)
    {
        HAL_GPIO_WritePin(brakes_port, brakes_pin, GPIO_PIN_SET);
    }
    else if (brakes_flag == 0)
    {
        HAL_GPIO_WritePin(brakes_port, brakes_pin, GPIO_PIN_RESET);
    }
    switch (Speed_level)
    {
    case 0:
        Speed_map = 0.4;
        break;
    case 1:
        Speed_map = 0.8;
        break;
    case 2:
        Speed_map = 1.2;
        break;
    default:
        Speed_map = 0;
        break;
    }
    if (if_motor_start == 1)
    {
        shooter_motor->set_rpm(Speed_map * xbox_msgs.joyLVert_map * max_shooter_speed);
        lifting_motor->set_rpm(Speed_map * xbox_msgs.joyRVert_map * max_lifting_speed);
        turn_motor->set_rpm(Speed_map * xbox_msgs.joyRHori_map * max_turn_speed);
        pithcer_motor->set_rpm(-(Speed_map * xbox_msgs.trigLT_map - xbox_msgs.trigRT_map) * max_pithcer_speed);
    }
    else if (if_motor_start == 0)
    {
        lifting_motor->set_rpm(0.0f);
        turn_motor->set_rpm(0.0f);
        shooter_motor->set_rpm(0.0f);
        pitching_motor->set_rpm(0.0f);
    }
}

void UserCtrl_xbox::add_motor(power_motor *lifting_motor_, power_motor *turn_motor_, power_motor *shooter_motor_, power_motor *pitching_motor_)
{
    lifting_motor = lifting_motor_;
    turn_motor = turn_motor_;
    shooter_motor = shooter_motor_;
    pitching_motor = pitching_motor_;
}

void UserCtrl_xbox::add_trigger(GPIO_TypeDef *clamp_port_, uint16_t clamp_pin_, GPIO_TypeDef *shooter_port_, uint16_t shooter_pin_, GPIO_TypeDef *brakes_port_, uint16_t brakes_pin_)
{
    clamp_port = clamp_port_;
    clamp_pin = clamp_pin_;
    shooter_port = shooter_port_;
    shooter_pin = shooter_pin_;
    brakes_port = brakes_port_;
    brakes_pin = brakes_pin_;
}
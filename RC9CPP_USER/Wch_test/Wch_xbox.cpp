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
        &lock_flag,
        1,
        ButtonActionType::Toggle,
        nullptr};

    btnLBConfig = {
        &xbox_msgs.btnLB,
        &xbox_msgs.btnLB_last,
        &shoot_flag,
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
}

void UserCtrl_xbox::btn_scan()
{
    handleButton(btnAConfig);
    handleButton(btnRBConfig);
    handleButton(btnLBConfig);
    handleButton(btnRSConfig);
}

UserCtrl_xbox::UserCtrl_xbox(float full_speed_, float full_w_) : full_speed(full_speed_), full_w(full_w_)
{
    btnconfig_init();
}

void UserCtrl_xbox::process_data()
{
    btn_scan();
    joymap_compute();
    if (this->xbox_msgs.btnDirUp == 1)
    {
        pitching_motor->set_rpm(-0.25*max_pitching_speed);
    }
    else if (this->xbox_msgs.btnDirDown == 1)
    {
        pitching_motor->set_rpm(0.25*max_pitching_speed);
    }
    else
    {
        pitching_motor->set_rpm(0.0f);
    }

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

    if (if_motor_start == 1)
    {
        if (lock_flag == 0)
        {
            shooter_motor->set_rpm(-xbox_msgs.trigRT_map * max_shooter_speed);
            shooter_motor->set_rpm(xbox_msgs.trigLT_map * max_shooter_speed);
            lifting_motor->set_rpm(xbox_msgs.joyRVert_map * max_lifting_speed);

            Vector2D worldvel_(x_planner.plan(full_speed * xbox_msgs.joyLHori_map), y_planner.plan(full_speed * xbox_msgs.joyLVert_map));
            set_RobotVel(worldvel_, priocode);
            set_RobotW(-w_planner.plan(full_w * xbox_msgs.joyRHori_map), priocode);
        }
        else if (lock_flag == 1)
        {
            shooter_motor->set_rpm(-xbox_msgs.trigRT_map * max_shooter_speed);
            shooter_motor->set_rpm(xbox_msgs.trigLT_map * max_shooter_speed);
            lifting_motor->set_rpm(xbox_msgs.joyRVert_map * max_lifting_speed);
            turn_motor->set_rpm(xbox_msgs.joyRHori_map * max_turn_speed);

            Vector2D worldvel_(x_planner.plan(full_speed * xbox_msgs.joyLHori_map), y_planner.plan(full_speed * xbox_msgs.joyLVert_map));
            set_RobotVel(worldvel_, priocode);
        }
    }
    else if (if_motor_start == 0)
    {
        lifting_motor->set_rpm(0.0f);
        turn_motor->set_rpm(0.0f);
        shooter_motor->set_rpm(0.0f);
        pitching_motor->set_rpm(0.0f);

        set_RobotVel(Vector2D(0.0f, 0.0f), priocode);
        set_RobotW(0.0f, priocode);

        x_planner.reset_speed();
        y_planner.reset_speed();
        w_planner.reset_speed();
    }
}

void UserCtrl_xbox::add_motor(power_motor *lifting_motor_, power_motor *turn_motor_, power_motor *shooter_motor_, power_motor *pitching_motor_)
{
    lifting_motor = lifting_motor_;
    turn_motor = turn_motor_;
    shooter_motor = shooter_motor_;
    pitching_motor = pitching_motor_;
}

void UserCtrl_xbox::add_trigger(GPIO_TypeDef *clamp_port_, uint16_t clamp_pin_, GPIO_TypeDef *shooter_port_, uint16_t shooter_pin_)
{
    clamp_port = clamp_port_;
    clamp_pin = clamp_pin_;
    shooter_port = shooter_port_;
    shooter_pin = shooter_pin_;
}
void UserCtrl_xbox::init_plan(float max_xy_acc, float max_w_acc)
{
    x_planner.reset(0.0f, max_xy_acc);
    y_planner.reset(0.0f, max_xy_acc);
    w_planner.reset(0.0f, max_w_acc);
}
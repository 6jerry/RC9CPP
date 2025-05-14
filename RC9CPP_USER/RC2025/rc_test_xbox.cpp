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
        &auto_mode,
        1,
        ButtonActionType::Toggle,
        nullptr};

    btnXConfig = {
        &xbox_msgs.btnX,
        &xbox_msgs.btnX_last,
        &auto_revert,
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

    btnDirUpConfig = {
        &xbox_msgs.btnDirUp,
        &xbox_msgs.btnDirUp_last,
        &yun_trigger,
        1,
        ButtonActionType::Toggle,
        nullptr};
    btnDirDownConfig = {
        &xbox_msgs.btnDirDown,
        &xbox_msgs.btnDirDown_last,
        &pithcer_status,
        1,
        ButtonActionType::Increment,
        nullptr};

    btnDirLeftConfig = {
        &xbox_msgs.btnDirLeft,
        &xbox_msgs.btnDirLeft_last,
        &lifter_status,
        9,
        ButtonActionType::Increment,
        nullptr};

    btnShareConfig = {
        &xbox_msgs.btnShare,
        &xbox_msgs.btnShare_last,
        &shoot_yunball,
        1,
        ButtonActionType::Toggle,
        nullptr};
}

void yun_ball_xbox::btn_scan()
{
    handleButton(btnAConfig);
    handleButton(btnRBConfig);
    handleButton(btnYConfig);
    handleButton(btnXConfig);
    handleButton(btnLBConfig);
    handleButton(btnDirUpConfig);
    handleButton(btnDirDownConfig);
    handleButton(btnDirLeftConfig);
    handleButton(btnShareConfig);
}

yun_ball_xbox::yun_ball_xbox()
{
    btnconfig_init();
}

void yun_ball_xbox::process_data()
{
    btn_scan();
    joymap_compute();

    if (if_motor_start == 1)
    {
        if (shoot_yunball == 0) // 射球模式
        {

            if (yun_trigger == 1)
            {

                HAL_GPIO_WritePin(yun_port, yun_pin, GPIO_PIN_SET);
            }
            else if (yun_trigger == 0)
            {

                HAL_GPIO_WritePin(yun_port, yun_pin, GPIO_PIN_RESET);
            }
            // 手动模式
            if (auto_mode == 0)
            {
                auto_shooter->trigger_flag = trigger_start;
                auto_shooter->shooter_flag = shooter_trigger;

                lifter_motor->set_rpm(xbox_msgs.joyRVert_map * max_lifter_speed);
                turn_motor->set_rpm(-xbox_msgs.joyRHori_map * max_turn_speed);
                // auto_shooter->set_pitcher_mode(pitcher_hand);
                // auto_shooter->shooter_info.hand_pitcher_rpm = -(xbox_msgs.trigLT_map - xbox_msgs.trigRT_map) * max_pithcer_speed;
                auto_shooter->set_shooter_mode(shooter_hand);
                auto_shooter->shooter_info.hand_shooter_rpm = xbox_msgs.joyLVert_map * max_shooter_speed;
            }
            // 自动模式
            else if (auto_mode == 1)
            {
                if (lifter_status == 0)
                {
                    auto_shooter->set_shooter_mode(shooter_stop);
                }
                else
                {
                    if (auto_flag == 0)
                    {
                        auto_shooter->set_allAuto(lifter_status - 1);
                        auto_flag = 1;
                    }

                    if (auto_shooter->isfinish())
                    {
                        lifter_status = 0;
                        auto_flag = 0;
                    }
                }
            }
        }
        else if (shoot_yunball == 1)
        {
            if (yun_trigger == 1)
            {
                yunball->start_multi_yun();
            }
            else if (yun_trigger == 0)
            {
                yunball->stop();
            }
        }
    }
    else if (if_motor_start == 0)
    {
        lifter_motor->set_rpm(0.0f);
        turn_motor->set_rpm(0.0f);
        auto_shooter->set_pitcher_mode(pitcher_stop);
        auto_shooter->set_shooter_mode(shooter_stop);
    }
}
void yun_ball_xbox::add_motor(power_motor *lfter_motor_, power_motor *turn_motor_)
{
    lifter_motor = lfter_motor_;
    turn_motor = turn_motor_;
}

void yun_ball_xbox::add_trigger(uint16_t yun_pin_, GPIO_TypeDef *yun_port_)
{

    yun_port = yun_port_;
    yun_pin = yun_pin_;
}

void yun_ball_xbox::add_serial_studio(serialStudio *serial_studio_)
{
    serial_studio = serial_studio_;
}

void yun_ball_xbox::add_auto_shooter(AutoShooter *auto_shooter_)
{
    auto_shooter = auto_shooter_;
}
void yun_ball_xbox::add_yunball(auto_yunball *yunball_)
{
    yunball = yunball_;
}
#include "ball_shooter.h"

void BallShooter::process_data()
{

    real_dis = laser->get_distance();

    switch (workmode)
    {
    case shooter_current:
        pull_moter->set_current(target_current);
        break;
    case shooter_rpm:
        pull_moter->send_rpm(target_rpm);
        break;
    case shooter_dis:
        pull_moter->set_current(-pull_dis_control.increPID_Compute(real_dis));
        break;
    case shooter_stop:
        pull_moter->send_rpm(0.0f);
        break;

    case shooter_custom_rpm:
        pull_moter->set_rpm(target_rpm);
        break;

    default:
        break;
    }

    float send_datas[3] = {real_dis * 1000.0f, pull_moter->get_rpm(), target_dis * 1000.0f};
    sendFloatData(1, send_datas, 3);
}

void BallShooter::set_pull_dis(float dis)
{
    target_dis = dis;

    if (target_dis > max_dis)
    {
        target_dis = max_dis;
    }
    if (target_dis < min_dis)
    {
        target_dis = min_dis;
    }

    if (target_dis <= 0.0f)
    {
        target_dis = 0.0f;
    }

    pull_dis_control.increPID_setarget(target_dis);

    // pull_dis_control.setpoint = target_dis;
    workmode = shooter_dis;
}

float BallShooter::get_pull_dis()
{
    float pulldis = 0.0f;

    pulldis = max_dis - laser->get_distance();

    if (pulldis < 0.0f)
    {
        pulldis = 0.0f;
    }

    return pulldis;
}

void BallShooter::send_current(float current)
{
    target_current = current;
    workmode = shooter_current;
}

void BallShooter::send_rpm(float rpm)
{
    target_rpm = rpm;
    workmode = shooter_rpm;
}

void BallShooter::stop()
{
    workmode = shooter_stop;
}

void BallShooter::add_moter(power_motor *moter_)
{
    pull_moter = moter_;
}

void BallShooter::add_laser(imu *laser_)
{
    laser = laser_;
}

void BallShooter::set_ff(float ff_)
{
    pull_moter->set_ff_current(ff_);
}

void BallShooter::set_motor_rpm(float rpm)
{

    target_rpm = rpm;
    workmode = shooter_custom_rpm;
}

void ball_shooter_xbox::process_data()
{
    btn_scan();
    joymap_compute();

    ff_current = K * (shooter->get_pull_dis() * shooter->get_pull_dis()) + K * R * shooter->get_pull_dis();

    if (start_flag == 1)
    {
        switch (debug_mode)
        {
        case 0:
            shooter->set_motor_rpm((xbox_msgs.trigLT_map - xbox_msgs.trigRT_map) * max_rpm);
            shooter->set_ff(ff_current);
            break;
        case 1:
            shooter->send_rpm((xbox_msgs.trigLT_map - xbox_msgs.trigRT_map) * max_rpm);

            break;
        case 2:
            shooter->set_pull_dis((1 - xbox_msgs.trigLT_map) * max_dis);

            break;
        case 3:

            break;
        }
    }
    else
    {
        shooter->stop();
    }
}

void ball_shooter_xbox::btnconfig_init()
{

    btnBConfig = {
        &xbox_msgs.btnB,
        &xbox_msgs.btnB_last,
        &debug_mode,
        3,
        ButtonActionType::Increment,
        nullptr};

    btnXConfig = {
        &xbox_msgs.btnX,
        &xbox_msgs.btnX_last,
        &debug_mode,
        3,
        ButtonActionType::Decrement,
        nullptr};

    btnAConfig = {
        &xbox_msgs.btnA,
        &xbox_msgs.btnA_last,
        &start_flag,
        1,
        ButtonActionType::Toggle,
        nullptr};
}

void ball_shooter_xbox::add_shooter(BallShooter *shooter_)
{
    shooter = shooter_;
}

void ball_shooter_xbox::btn_scan()
{

    handleButton(btnAConfig);
    handleButton(btnXConfig);
    handleButton(btnBConfig);
}

ball_shooter_xbox::ball_shooter_xbox()
{
    btnconfig_init();
}
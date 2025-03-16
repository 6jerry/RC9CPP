#include "ball_shooter.h"

void BallShooter::process_data()
{

    real_dis = laser->get_distance() - min_dis;

    if (get_input_start())
    {
        set_pull_dis((1 - get_input_mapvalue()) * max_dis);

        pull_dis_control.increPID_setarget(target_dis);

        pull_moter->set_rpm(get_input_mapvalue() * max_rpm);
        pid_send_debuginfo(get_input_mapvalue() * max_rpm, pull_moter->get_rpm());
    }
    else
    {
        pull_moter->set_current(0.0f);
    }
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
}

void BallShooter::add_moter(power_motor *moter_)
{
    pull_moter = moter_;
}

void BallShooter::add_laser(imu *laser_)
{
    laser = laser_;
}
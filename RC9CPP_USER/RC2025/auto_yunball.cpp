#include "auto_yunball.h"

void auto_yunball::process_data()
{
    scan_sensor();

    switch (workmode)
    {
    case yunball_init_locate:
        system_init();
        break;
    case yunball_remote_control:
        break;
    case yunball_standby:
        claw_close();
        break;
    case yunball_move_2_catch_point:
        move_2_catch_point();
        break;
    case yunball_move_2_throw_point:
        move_2_throw_point();
        break;
    default:
        break;
    }
}

void auto_yunball::scan_sensor()
{
    locate_flag = HAL_GPIO_ReadPin(locate_sensor_port, locate_sensor_pin);
    ball_flag = HAL_GPIO_ReadPin(ball_sensor_port, ball_sensor_pin);
}

void auto_yunball::claw_open()
{
    HAL_GPIO_WritePin(claw_port, claw_pin, GPIO_PIN_RESET);
}

void auto_yunball::claw_close()
{
    HAL_GPIO_WritePin(claw_port, claw_pin, GPIO_PIN_SET);
}

void auto_yunball::add_io(GPIO_TypeDef *locate_sensor_port_, uint16_t locate_sensor_pin_, GPIO_TypeDef *ball_sensor_port_, uint16_t ball_sensor_pin_, GPIO_TypeDef *claw_port_, uint16_t claw_pin_)
{
    locate_sensor_port = locate_sensor_port_;
    locate_sensor_pin = locate_sensor_pin_;
    ball_sensor_port = ball_sensor_port_;
    ball_sensor_pin = ball_sensor_pin_;
    claw_port = claw_port_;
    claw_pin = claw_pin_;
}

void auto_yunball::add_motor(power_motor *lift_motor_)
{
    lift_motor = lift_motor_;
}

void auto_yunball::system_init()
{
    if (locate_flag != 0)
    {
        lift_motor->set_rpm(-60.0f);
    }
    else if (locate_flag == 0)
    {
        lift_motor->set_rpm(0.0f);
        lift_motor->relocate_dis(0.0f);

        workmode = yunball_move_2_catch_point;
    }
}

void auto_yunball::move_2_catch_point()
{
    claw_open();

    lift_motor->set_dis_speedplan(catch_ball_dis, max_catch_speed, max_catch_acc, max_catch_dec, final_catch_speed);

    if (abs(catch_ball_dis - lift_motor->get_dis()) <= 10.0f && ball_flag == 0)
    {
        claw_close();
        time_cnt++;
        if (time_cnt >= 100)
        {
            workmode = yunball_move_2_throw_point;
            lift_motor->dis_speedplan_restart();
            time_cnt = 0;
        }
    }
}

void auto_yunball::move_2_throw_point()
{
    claw_close();

    lift_motor->set_dis_speedplan(throw_ball_dis, max_throw_speed, max_throw_acc, max_throw_dec, final_throw_speed);

    if (abs(throw_ball_dis - lift_motor->get_dis()) <= 3.0f)
    {
        claw_open();
        lift_motor->dis_speedplan_restart();
        workmode = yunball_move_2_catch_point;
    }
}

void auto_yunball::start_multi_yun()
{
    if (workmode == yunball_standby)
    {
        workmode = yunball_init_locate;
    }
}

void auto_yunball::stop()
{
    workmode = yunball_standby;
}

void auto_yunball_xbox::not_start()
{
    yunball->stop();
    lifter_motor->set_rpm(0.0f);
}

void auto_yunball_xbox::mode_2()
{
    yunball->stop();
    lifter_motor->set_rpm(xbox_msgs.joyRVert_map * 200.0f);
}

void auto_yunball_xbox::mode_3()
{
    yunball->start_multi_yun();
}

void auto_yunball_xbox::add_yunball(auto_yunball *yunball_)
{
    yunball = yunball_;
}

void auto_yunball_xbox::add_lifter(power_motor *lifter_motor_)
{
    lifter_motor = lifter_motor_;
}
#include "auto_yunball.h"

void auto_yunball::process_data()
{
    scan_sensor();
    switch (workmode)
    {
        case yunball_init_locate:
            system_init();
        break;
        case yunball_move_2_throw_point:
            move_2_throw_point();
        break;
        case yunball_move_2_catch_point:
            move_2_catch_point();
        break;
        case yunball_standby:
            lift_motor->dis_speedplan_restart();
            time_cnt = 0;
            time_flag = 0;
        break;
        case yunball_move_2_turn_point:
            move_2_turn_point();
        break;
        case yunball_turn_2_throw_point:
            turn_2_throw_point();
        break;
        case yunball_turn_back:
            turn_back();
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

void auto_yunball::add_motor(power_motor *lift_motor_, power_motor *turn_motor_)
{
    lift_motor = lift_motor_;
    turn_motor = turn_motor_;
}






void auto_yunball::system_init()
{
		lift_motor->set_dis_speedplan(500,1500,400,500,0);
		if (abs(500 - lift_motor->get_dis()) <= 10.0f)
		{
				lift_motor->dis_speedplan_restart();
				workmode = yunball_move_2_throw_point;
		}
}

void auto_yunball::move_2_throw_point()
{
    claw_close();
    lift_motor->set_dis_speedplan(850,2000,1500,1500,0);
    if (abs(750 - lift_motor->get_dis()) <= 50.0f)
    {
        claw_open();
        workmode = yunball_move_2_catch_point;
        lift_motor->dis_speedplan_restart();                     
    }
}

void auto_yunball::move_2_catch_point()
{
    lift_motor->set_dis_speedplan(300,2000,2000,2000,500);
    if (abs(300 - lift_motor->get_dis()) <= 20.0f)
    {
        lift_motor->set_rpm(0.0f);
        time_cnt++;
        if (time_cnt >= 1)
        {
            if(time_flag++ == 0) workmode = yunball_move_2_throw_point;
            else 
            {
                lift_motor->set_rpm(0.0f);
                workmode = yunball_standby;
                time_flag = 0;
            }
            lift_motor->dis_speedplan_restart();
            claw_close();
            time_cnt = 0;
        }
    }
}

void auto_yunball::move_2_turn_point()
{
    lift_motor->set_dis_speedplan(750,1500,1000,1000,0);
    if (abs(750 - lift_motor->get_dis()) <= 10.0f)
    {
        workmode = yunball_turn_2_throw_point;
        lift_motor->dis_speedplan_restart();
    }
}

void auto_yunball::turn_2_throw_point()
{
    turn_motor->set_pos_speedplan(-90.0f,30.0f,10.0f,10.0f,0.0f);
    if(abs(-90.0f - turn_motor->get_pos()) <= 5.0f)
    {
        turn_motor->set_rpm(0.0f);
        turn_motor->pos_speedplan_restart();
        time_cnt++;
        if(time_cnt>=50)
        {                    
            turn_motor->dis_speedplan_restart();
            claw_open();
        }
        if(time_cnt>=100)
        {
            workmode = yunball_turn_back;
            time_cnt = 0;
        }
    }
}

void auto_yunball::turn_back()
{
    turn_motor->set_pos_speedplan(90.0f,30.0f,10.0f,10.0f,0.0f);
    if(abs(90.0f - turn_motor->get_pos()) <= 5.0f)
    {
        turn_motor->set_rpm(0.0f);
        turn_motor->pos_speedplan_restart();
        workmode = yunball_standby;
        claw_close();
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
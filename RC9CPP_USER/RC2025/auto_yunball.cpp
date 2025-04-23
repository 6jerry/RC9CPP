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
            turn_motor->pos_speedplan_restart();
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
        case yunball_lift_motor_reset:
            lift_motor_reset();
        break;
        case yunball_turn_motor_reset:
            turn_motor_reset();
        break;
        case yunball_test_init:
            test_init();
        break;
        case yunball_test_throw:
            test_throw();
        break;
        case yunball_test_catch:
            test_catch();
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

void auto_yunball::push_open()
{
    HAL_GPIO_WritePin(push_port, push_pin, GPIO_PIN_RESET);
}

void auto_yunball::push_close()
{
    HAL_GPIO_WritePin(push_port, push_pin, GPIO_PIN_SET);
}

void auto_yunball::add_io(GPIO_TypeDef *locate_sensor_port_, uint16_t locate_sensor_pin_, GPIO_TypeDef *ball_sensor_port_, uint16_t ball_sensor_pin_, GPIO_TypeDef *claw_port_, uint16_t claw_pin_, GPIO_TypeDef *push_port_, uint16_t push_pin_)
{
    locate_sensor_port = locate_sensor_port_;
    locate_sensor_pin = locate_sensor_pin_;
    ball_sensor_port = ball_sensor_port_;
    ball_sensor_pin = ball_sensor_pin_;
    claw_port = claw_port_;
    claw_pin = claw_pin_;
    push_port = push_port_;
    push_pin = push_pin_;
}

void auto_yunball::add_motor(power_motor *lift_motor_, power_motor *turn_motor_)
{
    lift_motor = lift_motor_;
    turn_motor = turn_motor_;
}




/**********内部实现**********/

void auto_yunball::system_init()     //初始化
{
		lift_motor->set_dis_speedplan(500,1500,400,500,0);
		if (abs(500 - lift_motor->get_dis()) <= 10.0f)
		{
				lift_motor->dis_speedplan_restart();
				workmode = yunball_move_2_throw_point;
		}
}

void auto_yunball::move_2_throw_point()      //移动到抛球点
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

void auto_yunball::move_2_catch_point()      //移动到接球点
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
                workmode = yunball_move_2_turn_point;
                time_flag = 0;
            }
            lift_motor->dis_speedplan_restart();
            claw_close();
            time_cnt = 0;
        }
    }
}

void auto_yunball::move_2_turn_point()      //移动到转向点
{
    lift_motor->set_dis_speedplan(750,1500,1000,1000,0);
    if (abs(750 - lift_motor->get_dis()) <= 10.0f)
    {
        workmode = yunball_turn_2_throw_point;
        lift_motor->dis_speedplan_restart();
    }
}

void auto_yunball::turn_2_throw_point()      //转向到抛球点
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

void auto_yunball::turn_back()      //转向回到原位
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

uint8_t auto_yunball::lift_motor_reset()
{
    /*if(turn_motor->get_pos() < -10.0f)
    {
        return 0;
    }
    else
    {*/
        if(HAL_GPIO_ReadPin(locate_sensor_port, locate_sensor_pin) == GPIO_PIN_RESET)
        {
            lift_motor->set_rpm(0.0f);
            lift_motor->relocate_dis(0.0f);
            workmode = yunball_standby;
            return 1;
        }
        else
        {
            if(abs(lift_motor->get_dis()) >= 100.0f)
            {
                lift_motor->set_dis_speedplan(100,1500,1000,1000,-60);
            }
            else
            {
                lift_motor->set_rpm(-60.0f);
            }
            return 2;
        }
   //}
}

uint8_t auto_yunball::turn_motor_reset()
{
    static float tick = 0.0f;
    static float last_tick = 0.0f;
    static uint8_t derection = 1;
    static uint8_t step = 0;
    turn_motor->set_rpm(5.0f * derection);
    tick = HAL_GetTick();
    if(step == 0)
    {
        if(tick - last_tick >= 1000)
        { 
            step = 1;
            last_tick = tick;
            derection = -derection;
        }
    }
    else if(step == 1)
    {
        if(tick - last_tick >= 2000)
        {
            step = 2;
            last_tick = tick;
            derection = -derection;
        }
    }
    else if(step == 2)
    {
        turn_motor->set_rpm(0.0f);
        workmode = yunball_standby;
        return 2;
    }


    if(HAL_GPIO_ReadPin(ball_sensor_port, ball_sensor_pin) == GPIO_PIN_RESET)
    {
        turn_motor->set_rpm(0.0f);
        workmode = yunball_standby;
        step = 0;
        return 1;
    }
}

void auto_yunball::test_init()
{
            lift_motor->dis_speedplan_restart();
            workmode = yunball_test_throw;
}

void auto_yunball::test_throw()
{
    claw_open();
    push_close();
    last_tick = HAL_GetTick();
    workmode = yunball_test_catch;
}

void auto_yunball::test_catch()
{
		if(HAL_GetTick() - last_tick >= 200)
		{
			  push_open();
		}
    if(HAL_GetTick() - last_tick >= delay_tick)
    {
        claw_close();
        workmode = yunball_standby;
    }
}








/*******外部调用接口*******/

//开启自动运球
void auto_yunball::start_multi_yun()
{
    if (workmode == yunball_standby)
    {
        workmode = yunball_init_locate;
    }
}
//停止自动运球
void auto_yunball::stop()
{
    workmode = yunball_standby;
}

void auto_yunball::start_test_yun()
{
    if(workmode == yunball_standby)
    {
        workmode = yunball_test_init;
    }
}

void auto_yunball::lift_reset()
{
    if(workmode == yunball_standby)
    {
        workmode = yunball_lift_motor_reset;
    }
}

void auto_yunball::turn_reset()
{
    if(workmode == yunball_standby)
    {
        workmode = yunball_turn_motor_reset;
    }
}


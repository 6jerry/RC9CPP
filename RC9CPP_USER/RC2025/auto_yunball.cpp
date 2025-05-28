#include "auto_yunball.h"

uint32_t yunball_turn_tick = 0;
uint32_t yunball_back_tick = 0;

void auto_yunball::process_data()
{
    scan_sensor();
    switch (workmode)
    {
    case yunball_standby:
        lift_motor->dis_speedplan_restart();
        turn_motor->pos_speedplan_restart();
        time_cnt = 0;
        time_flag = 0;
        break;
    case yunball_turn:
        lift_motor->set_dis_speedplan(380, 500, 350, 350, 0);
        osDelay(300);
        turn_motor->set_pos_speedplan(-270.0f, 20.0f, 10.0f, 10.0f, 0.0f);
        if (((abs(-270.0f - turn_motor->get_pos_all()) <= 3.0f) && (abs(380 - lift_motor->get_dis())) <= 15.0f) || (timeout(yunball_turn_tick)))
        {
            turn_motor->set_rpm(0.0f);
            lift_motor->set_rpm(0.0f);
            lift_motor->dis_speedplan_restart();
            turn_motor->pos_speedplan_restart();
            osDelay(300);
            claw_open();
            osDelay(300);
            yunball_turn_tick = 0;
            workmode = yunball_back;
        }
        break;
    case yunball_back:
        turn_motor->set_pos_speedplan(-120.0f, 20.0f, 10.0f, 10.0f, 0.0f);
        osDelay(700);
        lift_motor->set_dis_speedplan(0, 300, 200, 200, 0);
        if (((abs(-120.0f - turn_motor->get_pos_all()) <= 3.0f) && (abs(0 - lift_motor->get_dis()) <= 15.0f)) || (timeout(yunball_back_tick)))
        {
            turn_motor->set_rpm(0.0f);
            turn_motor->pos_speedplan_restart();
            lift_motor->set_rpm(0.0f);
            lift_motor->dis_speedplan_restart();
            yunball_back_tick = 0;
            workmode = yunball_standby;
            claw_close();
        }
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


uint8_t auto_yunball::lift_motor_reset()
{
    if (HAL_GPIO_ReadPin(locate_sensor_port, locate_sensor_pin) == GPIO_PIN_RESET)
    {
        lift_motor->set_rpm(0.0f);
        lift_motor->relocate_dis(0.0f);
        workmode = yunball_standby;
        return 1;
    }
    else
    {
        if (abs(lift_motor->get_dis()) >= 100.0f)
        {
            lift_motor->set_dis_speedplan(100, 1500, 1000, 1000, -60);
        }
        else
        {
            lift_motor->set_rpm(-60.0f);
        }
        return 2;
    }
}

uint8_t auto_yunball::turn_motor_reset()
{
    static float tick = 0.0f;
    static float last_tick = 0.0f;
    static uint8_t derection = 1;
    static uint8_t step = 0;
    turn_motor->set_rpm(5.0f * derection);
    tick = HAL_GetTick();
    if (step == 0)
    {
        if (tick - last_tick >= 1000)
        {
            step = 1;
            last_tick = tick;
            derection = -derection;
        }
    }
    else if (step == 1)
    {
        if (tick - last_tick >= 2000)
        {
            step = 2;
            last_tick = tick;
            derection = -derection;
        }
    }
    else if (step == 2)
    {
        turn_motor->set_rpm(0.0f);
        workmode = yunball_standby;
        return 2;
    }

    if (HAL_GPIO_ReadPin(ball_sensor_port, ball_sensor_pin) == GPIO_PIN_RESET)
    {
        turn_motor->set_rpm(0.0f);
        workmode = yunball_standby;
        step = 0;
        return 1;
    }
}

void auto_yunball::test_init()
{
    /*turn_motor->set_pos_speedplan(-120.0f, 20.0f, 10.0f, 10.0f, 0.0f);
    if(abs(-120.0f - turn_motor->get_pos_all()) <= 3.0f)
    {
        lift_motor->dis_speedplan_restart();
        turn_motor->pos_speedplan_restart();
				osDelay(500);
        workmode = yunball_test_throw;
        last_tick_2 = HAL_GetTick();
    }*/
    workmode = yunball_test_throw;
    last_tick_2 = HAL_GetTick();
}

void auto_yunball::test_throw()
{
    claw_open();
    if (HAL_GetTick() - last_tick_2 >= 50)
    {
        push_close();
        last_tick = HAL_GetTick();
        workmode = yunball_test_catch;
    }
}

void auto_yunball::test_catch()
{
    static int cnt = 0;
    if (HAL_GetTick() - last_tick >= 200)
    {
        push_open();
    }
    if (HAL_GetTick() - last_tick >= delay_tick)
    {
        claw_close();
        workmode = yunball_standby;
        /*if(cnt > 0)
        {
            cnt = 0;
            workmode = yunball_standby;
        }
        else
        {
            osDelay(500);
            cnt++;
            workmode = yunball_test_init;
        }*/

    }
}

bool auto_yunball::start_put_ball()
{
    static uint8_t flag = 0;
    if (workmode == yunball_standby)
    {
        if(flag == 0)
        {
            workmode = yunball_turn;
            flag++;
        }
        else if(flag == 1)
        {
            flag = 0;
            return true;
        }
    }
    return false;
}

bool auto_yunball::put_ball()
{
    static uint8_t step = 0;
    switch (step)
    {
        case 0:
            if(workmode == yunball_standby)
            {
                workmode = yunball_test_init;
                step = 1;
            }
        break;
        case 1:
            if(workmode == yunball_standby)
            {
                workmode = yunball_turn;
                step = 2;
                return true;
            }
        break;
        default:
            return false;
        break;
    }
    return false;
}  

void auto_yunball::stop()
{
    workmode = yunball_standby;
}

bool auto_yunball::start_test_yun()
{
    static uint8_t flag = 0;
    if (workmode == yunball_standby)
    {
        if(flag == 0)
        {
            workmode = yunball_test_init;
            flag++;
        }
        else if(flag == 1)
        {
            flag=0;
            return true;
        }
    }
    return false;
}

void auto_yunball::lift_reset()
{
    lift_motor->relocate_dis(0.0f);
}


bool timeout(uint32_t &tick)
{
    if(tick == 0)    tick = HAL_GetTick();
    return (HAL_GetTick() - tick) > 2000;
}
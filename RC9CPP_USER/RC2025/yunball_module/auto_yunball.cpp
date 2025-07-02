#include "auto_yunball.h"

static uint8_t tile[4] = {0x00,0x00,0x80,0x7F};

void auto_yunball::process_data()
{
    if(flag != emergency_stop)
    {
        emergency_stop_flag = 0;
    }
    else 
    {
        // 信号量递减逻辑
        if(emergency_semaphore > 0) 
        {
            emergency_semaphore--;  // 每次循环递减1
        }
    }
    // 信号量状态判断
    if(emergency_semaphore <= 0 && flag == emergency_stop) 
    {
        flag = static_flag;  // 自动退出紧急状态
    }

    switch(flag)
    {
        case static_flag:
            turn_motor->set_rpm(-get_speed_turn * max_turn_speed);
            lift_motor->set_rpm(get_speed_lift * max_lift_speed);
            break;
        case yunball_flag:
            yunball();
            break;
        case putball_flag:
            putball();
            break;
        case stop_flag:
            osDelay(200);
            flag = static_flag;
            break;
        case emergency_stop:
            if(emergency_stop_flag++ < 10)
            {
                turn_motor->set_rpm(0.0f);
                lift_motor->set_rpm(0.0f);
            }
        default:
            break;
    }
}

void auto_yunball::add_io(GPIO_TypeDef *claw_port_, uint16_t claw_pin_, GPIO_TypeDef *push_port_, uint16_t push_pin_, GPIO_TypeDef *turn_port_, uint16_t turn_pin_)
{
    claw_port = claw_port_;
    claw_pin = claw_pin_;
    push_port = push_port_;
    push_pin = push_pin_;
    turn_port = turn_port_;
    turn_pin = turn_pin_;
}

void auto_yunball::add_motor(m3508p *turn_motor_, m3508p *lift_motor_)
{
    turn_motor = turn_motor_;
    lift_motor = lift_motor_;
}

//外部接口函数
bool auto_yunball::start_yunball()
{
    if(flag == static_flag) {flag = yunball_flag;}
    if(flag == stop_flag) {return true;}
    return false;
}
bool auto_yunball::start_putball()
{
    if(flag == static_flag) {flag = putball_flag;}
    if(flag == stop_flag) {return true;}
    return false;
}

void auto_yunball::stop()
{
    // 重置信号量为正数
    emergency_semaphore = 10;  // 这个数值控制紧急状态的持续时间
    flag = emergency_stop;
}

void auto_yunball::add_shooter(AutoShooter *shooter_)
{
    shooter = shooter_;
}

void auto_yunball::control_turn_motor(float speed_){get_speed_turn = speed_;}
void auto_yunball::control_lift_motor(float speed_){get_speed_lift = speed_;}
void auto_yunball::control_claw(bool if_open){if(flag == static_flag)set_claw(if_open);}
void auto_yunball::control_push(bool if_push){if(flag == static_flag)set_push(if_push);}



//内部实现函数
void auto_yunball::yunball()
{
    set_claw(true);
    set_push(true);
    osDelay(200); 
    set_push(false);
    osDelay(300);
    set_claw(false);
    flag = stop_flag;
}
void auto_yunball::putball()
{
    turn_motor->pos_speedplan_restart();
    turn_motor->set_pos_speedplan(-180.0f, 30.0f, 40.0f, 30.0f, 5.0f);            //旋转到-180度
    lift_motor->dis_speedplan_restart();
    lift_motor->set_dis_speedplan(-12.5f, 180.0f, 180.0f, 180.0f, 0.0f);
    shooter->set_lift(0.06f);
    while((abs(turn_motor->rcurrent) < 18000.0f) || (abs(lift_motor->get_dis()-(-12.5f)) > 0.5f)) {    //等待转动完成
        osDelay(1);
    }
    turn_motor->pos_speedplan_restart();                                    //复位
    turn_motor->set_rpm(0.0f);
    lift_motor->dis_speedplan_restart();    
	lift_motor->set_rpm(0.0f);
    turn_motor->all_pos = -180.0f;
    set_claw(true);                                                         //放球
    osDelay(300);
    shooter->set_lift(0.17f);                                 //拉皮筋
    osDelay(500);
    turn_motor->set_pos_speedplan(0.0f, 30.0f, 50.0f, 30.0f, 0.0f);       //旋转到0度
    osDelay(400);   
    lift_motor->set_dis_speedplan(0.0f, 180.0f, 180.0f, 180.0f, 0.0f);                                                   //下降  
    osDelay(300);
    shooter->set_lift(0.015f);                                //放皮筋
    while(abs(lift_motor->get_dis()-0.0f) > 0.5f) {    //等待转动完成
        osDelay(1);
    }        
    lift_motor->dis_speedplan_restart();
    lift_motor->set_rpm(0.0f);
	set_claw(false);
    flag = stop_flag;
}


void auto_yunball::set_claw(bool if_open) {HAL_GPIO_WritePin(claw_port, claw_pin, if_open ? GPIO_PIN_RESET : GPIO_PIN_SET);}
void auto_yunball::set_push(bool if_push) {HAL_GPIO_WritePin(push_port, push_pin, if_push ? GPIO_PIN_SET : GPIO_PIN_RESET);}
#include "auto_yunball.h"

static uint8_t tile[4] = {0x00, 0x00, 0x80, 0x7F};

void auto_yunball::process_data()
{
    if (flag != emergency_stop)
    {
        emergency_stop_flag = 0;
    }
    else
    {
        // 信号量递减逻辑
        if (emergency_semaphore > 0)
        {
            emergency_semaphore--; // 每次循环递减1
        }
    }
    // 信号量状态判断
    if (emergency_semaphore <= 0 && flag == emergency_stop)
    {
        flag = static_flag; // 自动退出紧急状态
    }

    switch (flag)
    {
    case static_flag:
        turn_motor->set_rpm(-get_speed_turn * max_turn_speed);
        lift_motor->set_rpm(get_speed_lift * max_lift_speed);
        turn_motor->all_pos = encoder_for_yunball->get_all_angle();
        turn_motor->pos_sum = encoder_for_yunball->get_angle();
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
        if (emergency_stop_flag++ < 10)
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

// 外部接口函数
bool auto_yunball::start_yunball()
{
    if (flag == static_flag)
    {
        flag = yunball_flag;
    }
    if (flag == stop_flag)
    {
        return true;
    }
    return false;
}
bool auto_yunball::start_putball()
{
    if (flag == static_flag)
    {
        flag = putball_flag;
    }
    if (flag == stop_flag)
    {
        return true;
    }
    return false;
}

void auto_yunball::stop()
{
    // 重置信号量为正数
    emergency_semaphore = 10; // 这个数值控制紧急状态的持续时间
    flag = emergency_stop;
}

void auto_yunball::add_shooter(AutoShooter *shooter_)
{
    shooter = shooter_;
}

void auto_yunball::add_encoder(Encoder *encoder_)
{
    encoder_for_yunball = encoder_;
}

void auto_yunball::control_turn_motor(float speed_) { get_speed_turn = speed_; }
void auto_yunball::control_lift_motor(float speed_) { get_speed_lift = speed_; }
void auto_yunball::control_claw(bool if_open)
{
    if (flag == static_flag)
        set_claw(if_open);
}
void auto_yunball::control_push(bool if_push)
{
    if (flag == static_flag)
        set_push(if_push);
}

// 内部实现函数
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
    static uint8_t putball_step = 0;
    static uint32_t step_start_time = 0;
    uint32_t current_time = HAL_GetTick();
    
    switch(putball_step)
    {
        case 0: // 初始化移动
            lift_motor->set_dis_speedplan(liftTo_dis, 180.0f, 180.0f, 180.0f, 0.0f);
            turn_motor->set_pos_speedplan(turnTo_angle, 40.0f, 40.0f, 30.0f, 0.0f);
            step_start_time = current_time;
            putball_step = 1;
            break;
            
        case 1: // 等待移动完成
            turn_motor->all_pos = encoder_for_yunball->get_all_angle();
            turn_motor->pos_sum = encoder_for_yunball->get_angle();
            if(turn_motor->pos_speed_plan.isFinished() && lift_motor->dis_speed_plan.isFinished())
            {
                turn_motor->pos_speedplan_restart();
                turn_motor->set_rpm(0.0f);
                lift_motor->dis_speedplan_restart();
                lift_motor->set_rpm(0.0f);
                step_start_time = current_time;
                putball_step = 2;
            }
            else if(current_time - step_start_time > 4000) // 4秒超时
            {
                putball_step = 7; // 跳转到结束步骤
            }
            break;
            
        case 2: // 延时500ms
            if(current_time - step_start_time >= 500)
            {
                set_claw(true);
                step_start_time = current_time;
                putball_step = 3;
            }
            break;
            
        case 3: // 延时300ms后拉皮筋
            if(current_time - step_start_time >= 300)
            {
                shooter->set_lift(shooter_lift);
                step_start_time = current_time;
                putball_step = 4;
            }
            break;
            
        case 4: // 延时500ms后开始返回
            if(current_time - step_start_time >= 500)
            {
                turn_motor->set_pos_speedplan(turnBack_angle, 30.0f, 50.0f, 30.0f, 0.0f);
                step_start_time = current_time;
                putball_step = 5;
            }
            break;
            
        case 5: // 延时600ms后下降
            if(current_time - step_start_time >= 600)
            {
                lift_motor->set_dis_speedplan(liftBask_dis, 180.0f, 180.0f, 180.0f, 0.0f);
                step_start_time = current_time;
                putball_step = 6;
            }
            break;
            
        case 6: // 延时300ms后放皮筋并等待完成
            if(current_time - step_start_time >= 300)
            {
                shooter->set_lift(0.015f);
                turn_motor->all_pos = encoder_for_yunball->get_all_angle();
                turn_motor->pos_sum = encoder_for_yunball->get_angle();
                if(turn_motor->pos_speed_plan.isFinished() && lift_motor->dis_speed_plan.isFinished())
                {
                    putball_step = 7;
                }
                else if(current_time - step_start_time > 3300) // 3秒超时
                {
                    putball_step = 7;
                }
            }
            break;
            
        case 7: // 结束清理
            lift_motor->dis_speedplan_restart();
            lift_motor->set_rpm(0.0f);
            lift_motor->dis_sum = liftBask_dis;
            turn_motor->pos_speedplan_restart();
            turn_motor->set_rpm(0.0f);
            set_claw(false);
            putball_step = 0; // 重置状态机
            flag = stop_flag;
            break;
    }
}

void auto_yunball::set_claw(bool if_open) { HAL_GPIO_WritePin(claw_port, claw_pin, if_open ? GPIO_PIN_RESET : GPIO_PIN_SET); }
void auto_yunball::set_push(bool if_push) { HAL_GPIO_WritePin(push_port, push_pin, if_push ? GPIO_PIN_SET : GPIO_PIN_RESET); }
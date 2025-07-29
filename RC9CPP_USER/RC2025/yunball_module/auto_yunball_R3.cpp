#include "auto_yunball_R3.h"

#define DEBOUNCE_DELAY 40 // 消抖时间40ms
static uint32_t last_trigger_time = 0;
static bool button_pressed = false;

static uint32_t debounce_start = 0;
constexpr uint32_t DEBOUNCE_THRESHOLD = 40; // 消抖时间40ms


void AutoYunballR3::process_data()
{
    test = HAL_GPIO_ReadPin(put_port_in, put_pin_in);

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

    switch(flag)
    {
        case init_flag:
            putball_motor->set_rpm(100.0f);
            if(HAL_GPIO_ReadPin(put_port_in, put_pin_in) == GPIO_PIN_RESET)
            {
                // 首次检测到低电平且未标记已按下
                if(!button_pressed)
                {
                    last_trigger_time = HAL_GetTick();
                    button_pressed = true;
                }
                else if(HAL_GetTick() - last_trigger_time > DEBOUNCE_DELAY)
                {
                    // 执行电机操作
                    putball_motor->set_current(0.0f);
                    putball_motor->relocate_dis(-5.0f);
                    putball_motor->set_rpm(-300.0f);
                    while(putball_motor->get_dis() > -16.0f)
                    {
                        osDelay(1);
                    }
                    putball_motor->set_rpm(0.0f);
                    flag = static_flag;
                }
            }
            else
            {
                // 检测到高电平时重置状态
                button_pressed = false;
                last_trigger_time = 0;
            }
            break;
        case static_flag:
            if(abs(get_speed_put) < 0.05f)
            {putball_motor->set_rpm(0.0f);}
            else
            {putball_motor->set_rpm(get_speed_put * put_speed);}
            break;
        case yunball_flag:
            yunball();
            flag = stop_flag;
            break;
        case putball_flag:
            putball();
            flag = stop_flag;
            break;
        case in_or_out_flag:
            set_out(if_out);
            flag = stop_flag;
            break;
        case yun_and_put_flag:
            yunball();
            osDelay(1000);
            yunball();
            putball();
            flag = stop_flag;
            break;
        case double_yun_flag:
            yunball();
            osDelay(1000);
            yunball();
            flag = stop_flag;
            break;
        case stop_flag:
            osDelay(200);
            flag = static_flag; // 自动退出停止状态
            break;
        case emergency_stop:
            if (emergency_stop_flag++ < 10)
            {
                putball_motor->set_current(0.0f);
            }
            break;
        default:
            break;
    }
}

void AutoYunballR3::add_motor(power_motor *putball_motor_)
{
    putball_motor = putball_motor_;
}

void AutoYunballR3::add_io(GPIO_TypeDef *put_port_in_, uint16_t put_pin_in_, GPIO_TypeDef *put_port_out_, uint16_t put_pin_out_, GPIO_TypeDef *push_port_, uint16_t push_pin_, GPIO_TypeDef *claw_port_, uint16_t claw_pin_)
{
    put_port_in = put_port_in_;
    put_pin_in = put_pin_in_;
    put_port_out = put_port_out_;
    put_pin_out = put_pin_out_;
    push_port = push_port_;
    push_pin = push_pin_;
    claw_port = claw_port_;
    claw_pin = claw_pin_;
}


// 外部接口函数
bool AutoYunballR3::if_is_finish()  {return flag ==  static_flag;}
bool AutoYunballR3::if_enable_shoot()  {return putball_motor->get_dis() < -18.0f;}

bool AutoYunballR3::in_or_out(bool if_out_)
{
    if(flag == static_flag)
    {
        if_out = if_out_;
        flag = in_or_out_flag;
    }
    if(flag == stop_flag)
    {
        return true;
    }
    return false;
}

bool AutoYunballR3::reload_motor()
{
    if (flag == static_flag)
    {
        flag = init_flag;
    }
    if (flag == stop_flag)
    {
        return true;
    }
    return false;
}

bool AutoYunballR3::start_yunball()
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
bool AutoYunballR3::start_putball()
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

bool AutoYunballR3::yun_and_put()
{
    if(flag == static_flag)
    {
        flag = yun_and_put_flag;
    }
    if(flag == stop_flag)
    {
        return true;
    }
    return false;
}

bool AutoYunballR3::double_yun()
{
    if(flag == static_flag)
    {
        flag = double_yun_flag;
    }
    if(flag == stop_flag)
    {
        return true;
    }
    return false;
}

void AutoYunballR3::stop()
{
    // 重置信号量为正数
    emergency_semaphore = 10; // 这个数值控制紧急状态的持续时间
    flag = emergency_stop;
}

void AutoYunballR3::control_put_motor(float speed_map) {get_speed_put = speed_map;}
void AutoYunballR3::control_claw(bool if_open) {if(flag == static_flag) set_claw(if_open);}
void AutoYunballR3::control_push(bool if_push) {if(flag == static_flag) set_push(if_push);}


// 内部实现函数
void AutoYunballR3::set_claw(bool if_open) {HAL_GPIO_WritePin(claw_port, claw_pin, if_open ? GPIO_PIN_SET : GPIO_PIN_RESET);}
void AutoYunballR3::set_push(bool if_push) {HAL_GPIO_WritePin(push_port, push_pin, if_push ? GPIO_PIN_SET : GPIO_PIN_RESET);}

void AutoYunballR3::set_out(bool if_out)
{
    if(if_out)
    {
        putball_motor->set_rpm(-200.0f);
        while(putball_motor->get_dis() > -22.0f)
        {
            osDelay(1);
        }
        putball_motor->set_rpm(0.0f);
    }
    else
    {
        putball_motor->set_rpm(200.0f);
        while(true)
        {
            GPIO_PinState current_state = HAL_GPIO_ReadPin(put_port_in, put_pin_in);
            
            // 消抖核心逻辑
            if(current_state == GPIO_PIN_RESET) {
                if(debounce_start == 0) {
                    debounce_start = HAL_GetTick();
                } else if(HAL_GetTick() - debounce_start > DEBOUNCE_THRESHOLD) {
                    break; // 确认稳定低电平后退出循环
                }
            } else {
                debounce_start = 0; // 高电平重置计时
            }

            if(putball_motor->get_dis() > -9.0f){
                putball_motor->set_rpm(50.0f);
            }
            osDelay(1);
        }
        putball_motor->set_current(0.0f);
        putball_motor->relocate_dis(-5.0f);
    }
}

void AutoYunballR3::putball()
{
    putball_motor->set_rpm(200.0f);
    while(true)
    {
        GPIO_PinState current_state = HAL_GPIO_ReadPin(put_port_in, put_pin_in);
        
        // 消抖核心逻辑
        if(current_state == GPIO_PIN_RESET) {
            if(debounce_start == 0) {
                debounce_start = HAL_GetTick();
            } else if(HAL_GetTick() - debounce_start > DEBOUNCE_THRESHOLD) {
                break; // 确认稳定低电平后退出循环
            }
        } else {
            debounce_start = 0; // 高电平重置计时
        }

        if(putball_motor->get_dis() > -9.0f){
            putball_motor->set_rpm(50.0f);
        }
        osDelay(1);
    }
    putball_motor->set_current(0.0f);
    putball_motor->relocate_dis(-5.0f);
    osDelay(200);
    set_claw(true);

}

void AutoYunballR3::yunball()
{
    if(putball_motor->get_dis() > -24.0f)
    {    
        putball_motor->set_rpm(-200.0f);
        while(HAL_GPIO_ReadPin(put_port_out, put_pin_out) == GPIO_PIN_SET)
        {
            if(putball_motor->get_dis() < -22.0f){putball_motor->set_rpm(-50.0f);}
            osDelay(1);
        }
        putball_motor->set_rpm(0.0f);
				putball_motor->relocate_dis(-26.0f);
        osDelay(200);
    }
    set_claw(true);
    set_push(true);
    osDelay(test_time1);
    set_push(false);
    osDelay(test_time2);
    set_claw(false);
}

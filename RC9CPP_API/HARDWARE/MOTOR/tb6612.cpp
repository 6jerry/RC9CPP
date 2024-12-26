#include "tb6612.h"

void tb6612::set_rpm(float power_motor_rpm)
{
    target_rpm = power_motor_rpm;
}

float tb6612::get_rpm()
{
    return now_rpm;
}

void tb6612::process_data()
{
    // set_vcurrent(200);
    calc_rpm();
    rpm_control.increPID_setarget(target_rpm * gear_ratio);
    vc = int16_t(rpm_control.increPID_Compute(now_rpm * gear_ratio));
    //vc = 100;
    set_vcurrent(vc);
}
tb6612::tb6612(TIM_HandleTypeDef *pwm_tim_, uint32_t pwm_channel_, TIM_HandleTypeDef *encoder_tim_, GPIO_TypeDef *dir_port1_, uint16_t GPIO_Pin1_, GPIO_TypeDef *dir_port2_, uint16_t GPIO_Pin2_, uint8_t gear_ratio_, uint8_t encoder_polse_) : pwm_tim(pwm_tim_), pwm_channel(pwm_channel_), encoder_tim(encoder_tim_), dir_port1(dir_port1_), dir_port2(dir_port2_), gear_ratio(gear_ratio_), rpm_control(0.27f, 0.086f, 0.0086f, 86.0f, 1000.0f, 5.0f), encoder_polse(encoder_polse_), GPIOPin1(GPIO_Pin1_), GPIOPin2(GPIO_Pin2_)
{
    polse_2_rpm = 1.0f / ((float)gear_ratio * (float)encoder_polse * 4.0f);
}

void tb6612::init()
{
    HAL_TIM_PWM_Start(pwm_tim, pwm_channel);
    HAL_TIM_Encoder_Start(encoder_tim, TIM_CHANNEL_ALL);
}

void tb6612::calc_rpm()
{
    uint32_t current_time = HAL_GetTick();
    test_nowtime = (float)current_time;
    if (previous_time != 0)
    {
        delta_time = (float)(current_time - previous_time) / 1000.0f;
    }
    current_count = __HAL_TIM_GET_COUNTER(encoder_tim);
    test_nowcount = (float)current_count;

    if (last_count < overflow_min && current_count > overflow_max)
    {
        delta_count = -(last_count + max_count - current_count);
    }
    else if (last_count > overflow_max && current_count < overflow_min)
    {
        delta_count = current_count + max_count - last_count;
    }
    else
    {
        delta_count = current_count - last_count;
    }

    test_deltacount = (float)delta_count;
    if (delta_time != 0.0f)
    {
        now_rpm = (((float)delta_count * polse_2_rpm) / delta_time) * 60.0f;
    }

    previous_time = current_time;
    last_count = current_count;

    odom_sum += ((float)delta_count * polse_2_rpm) * wheel_perimeter;
}

void tb6612::set_vcurrent(int16_t vcurrent_)
{
    if (vcurrent_ > 0)
    {
        HAL_GPIO_WritePin(dir_port1, GPIOPin1, GPIO_PIN_SET);
        HAL_GPIO_WritePin(dir_port2, GPIOPin2, GPIO_PIN_RESET);
        ccr = vcurrent_;
    }
    else
    {
        HAL_GPIO_WritePin(dir_port1, GPIOPin1, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(dir_port2, GPIOPin2, GPIO_PIN_SET);
        ccr = -vcurrent_;
    }

    __HAL_TIM_SET_COMPARE(pwm_tim, pwm_channel, ccr);
}

float tb6612::get_odom()
{
    return odom_sum;
}
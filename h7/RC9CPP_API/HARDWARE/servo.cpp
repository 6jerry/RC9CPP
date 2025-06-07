#include "servo.h"

void servo::init()
{
    HAL_TIM_PWM_Start(pwm_tim, pwm_channel);
}

servo::servo(TIM_HandleTypeDef *pwm_tim_, uint32_t pwm_channel_, uint16_t min_ccr_, uint16_t max_ccr_) : pwm_tim(pwm_tim_), pwm_channel(pwm_channel_), min_ccr(min_ccr_), max_ccr(max_ccr_)
{
}

void servo::set_ccr(uint16_t ccr_)
{
    __HAL_TIM_SET_COMPARE(pwm_tim, pwm_channel,ccr_);
}
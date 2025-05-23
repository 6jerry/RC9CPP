#ifndef SERVO_H
#define SERVO_H

#ifdef __cplusplus
extern "C"
{
#endif

#include "tim.h"
#include "Serial_device.h"
#ifdef __cplusplus
}
#endif

#ifdef __cplusplus

class servo
{
private:
    TIM_HandleTypeDef *pwm_tim;
    uint32_t pwm_channel;

    uint16_t min_ccr = 0, max_ccr = 0;

public:
    servo(TIM_HandleTypeDef *pwm_tim_, uint32_t pwm_channel_, uint16_t min_ccr_ = 50, uint16_t max_ccr_ = 250);
    void init();
    void set_ccr(uint16_t ccr_);
};

#endif
#endif
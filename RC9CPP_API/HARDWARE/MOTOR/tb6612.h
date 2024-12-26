#ifndef TB6612_H
#define TB6612_H

#ifdef __cplusplus
extern "C"
{
#endif
#include "TaskManager.h"
#include "SuperPID.h"
#include "motor.h"
#include "tim.h"
#include "gpio.h"

#ifdef __cplusplus
}
#endif

#ifdef __cplusplus
#define MAX_DELTA_COUNT 60000 // 设置为 2500，略大于计算值
#define overflow_min 300
#define overflow_max 65235
#define max_count 65535
class tb6612 : public ITaskProcessor, public power_motor
{
private:
    uint8_t gear_ratio = 45;
    TIM_HandleTypeDef *pwm_tim;
    uint32_t pwm_channel;
    TIM_HandleTypeDef *encoder_tim;

    GPIO_TypeDef *dir_port1;
    GPIO_TypeDef *dir_port2;

    uint16_t GPIOPin1, GPIOPin2;

    bool if_32bit_encoder = false;
    void set_vcurrent(int16_t vcurrent_);
    float wheel_perimeter = 0.1885f, odom_sum = 0.0f;

public:
    float get_rpm();
    void set_rpm(float power_motor_rpm);
    void process_data();
    tb6612(TIM_HandleTypeDef *pwm_tim_, uint32_t pwm_channel_, TIM_HandleTypeDef *encoder_tim_, GPIO_TypeDef *dir_port1_, uint16_t GPIO_Pin1_, GPIO_TypeDef *dir_port2_, uint16_t GPIO_Pin2_, uint8_t gear_ratio_ = 45, uint8_t encoder_polse_ = 13);
    void init();

    uint16_t ccr = 0; // 0~1000，默认定时器都是10khz频率,240mhz时钟的

    uint8_t encoder_polse = 13;

    float target_rpm = 0.0f, now_rpm = 0.0f;

    int16_t vc = 0;

    float delta_time = 0.0f;
    float polse_2_rpm = 0.0f; // 一个脉冲多少rpm
    uint32_t previous_time = 0, current_count = 0, last_count = 0;
    int32_t delta_count = 0;
    float test_nowcount = 0.0f, test_deltacount = 0.0f, test_nowtime = 0.0f;

    IncrePID rpm_control;
    void calc_rpm();
    float get_odom() override;
};

#endif
#endif
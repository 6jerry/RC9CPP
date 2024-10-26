#ifndef M3508_H
#define M3508_H
#ifdef __cplusplus
extern "C"
{
#endif

#include "can_device.h"
#include "pid.h"
#include "motor.h"

#ifdef __cplusplus
}
#endif

#ifdef __cplusplus

class m3508p : public CanDevice, public pid, public power_motor, public dji_motor // 动力电机版本的m3508
{
private:
    uint8_t gear_ratio = 1;

public:
    m3508p(uint8_t can_id, CAN_HandleTypeDef *hcan_, uint8_t gear_ratio = 19, float kp_ = 16.5f, float ki_ = 0.19f, float kd_ = 2.6f);

    int16_t motor_process() override;
    void can_update(uint8_t can_RxData[8]);
    float rtarget_angle = 0;
    float target_rpm = 0;

    // 动力电机通用接口
    float get_rpm();
    void set_rpm(float power_motor_rpm);
};

#endif

#endif
#ifndef M6020_H
#define M6020_H
#ifdef __cplusplus
extern "C"
{
#endif

#include "can_device.h"
#include "pid.h"
#include "motor.h"
#include <math.h>

#ifdef __cplusplus
}
#endif

#ifdef __cplusplus

class m6020s : public CanDevice, public servo_motor, public dji_motor // 动力电机版本的m3508
{
private:
    uint8_t gear_ratio = 1;

public:
    m6020s(uint8_t can_id, CAN_HandleTypeDef *hcan_, bool if_double_control_ = true, float kp_r = 270.0f, float ki_r = 1.8f, float kd_r = 6.0f, float kp_p = 0.0f, float ki_p = 0.0f, float kd_p = 0.0f);

    int16_t motor_process() override;
    void can_update(uint8_t can_RxData[8]);
    float target_angle = 0;
    float target_rpm = 0;
    int16_t target_v = 0;

    float real_angle = 0.0f;
    float angle_error = 0.0f;

    pid rpm_pid;

    bool if_double_control = true; // 是否使用双环控制，因为有些电机无法360度旋转导致无法调速度环
    pid pos_pid;

    float get_relative_pos();
    float get_absolute_pos();
    void set_relative_pos(float relative_pos_);
    void set_absolute_pos_multi(float absolute_pos_multi_);
    void set_absolute_pos_single(float absolute_pos_single_);
    void relocate(float new_zero_point);

    float convert_angle_to_signed(float current_angle);
};

#endif

#endif
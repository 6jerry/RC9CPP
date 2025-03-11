#ifndef SUPERPID_H
#define SUPERPID_H

#ifdef __cplusplus
extern "C"
{
#endif
#include <stdbool.h>
#include "TaskManager.h"
#include "Serial_device.h"
#ifdef __cplusplus
}
#endif

#ifdef __cplusplus
class superpid // 采用了梯形积分，微分先行，积分分离的改进版位置式pid
{
public:
    float kp = 0.0f;
    float ki = 0.0f;
    float kd = 0.0f;
    float setpoint = 0.0f;
    float i_out = 0.0f;
    float d_out = 0.0f;
    float p_out = 0.0f;
    float error_sum = 0.0f;
    float previous_error = 0.0f;
    float previous_input = 0.0f; // 用于微分先行
    float output = 0.0f;

    float output_limit = 0.0f;
    float deadzone = 0.0f;

    float integral_separation_threshold = 0.0f;
    float error = 0.0f;
    float sampling_period = 0.0f; // 新增采样周期，单位：秒

    bool max_output = false;
    uint32_t previous_time = 0;

    superpid(float kp_ = 0.0f, float ki_ = 0.0f, float kd_ = 0.0f, float output_limit_ = 0.0f, float deadzone_ = 0.0f, float integral_separation_threshold_ = 0.0f);

    void superPID_SetParameters(float kp, float ki, float kd);

    void config_all(float kp_, float ki_, float kd_, float output_limit_, float deadzone_, float integral_separation_threshold_);

    float superPID_Compute(float input);

    float superPID_ComputeError(float error_, float C_V); // 直接传入误差计算
};

class IncrePID // TD增量式
{
private:
    float r = 0.0f, expect = 0.0f, V1 = 0.0f, V2 = 0.0f, fh = 0.0f;
    float Ts = 0.0f;
    uint32_t previous_time = 0;

public:
    float error = 0.0f, last_error = 0.0f, lalast_error = 0.0f;
    float setpoint = 0.0f; // 经过TD处理过后的期望

    float output = 0.0f, last_output = 0.0f;
    float output_limit = 0.0f;
    float deadzone = 0.0f;
    float kp = 0.0f;
    float ki = 0.0f;
    float kd = 0.0f;

    float i_out = 0.0f;
    float d_out = 0.0f;
    float p_out = 0.0f;

    bool if_first_flag = true;

    float increPID_Compute(float input);
    void TD(); // 微分跟踪器
    void increPID_setarget(float target_);

    void calc();
    float increPID_Computerror(float error_);

    void
    increPID_SetParameters(float kp_, float ki_, float kd_, float r_);

    void config_all(float kp_, float ki_, float kd_, float r_, float output_limit_, float deadzone_);

    IncrePID(float kp_ = 0.0f, float ki_ = 0.0f, float kd_ = 0.0f, float r_ = 0.0f, float output_limit_ = 0.0f, float deadzone_ = 0.0f);
};

#endif
#endif

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
class superpid
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

    bool inertia_comp = false;
    uint32_t previous_time = 0;

    superpid(float kp, float ki, float kd, float output_limit, float deadzone,
             float integral_separation_threshold, bool if_inertia_comp = false);

    void superPID_SetParameters(float kp, float ki, float kd);

    float superPID_Compute(float input);

    float superPID_ComputeError(float error_, float C_V); // 直接传入误差计算
};
#endif
#endif

#ifndef PID_H
#define PID_H

#ifdef __cplusplus
extern "C"
{
#endif
#include <stdbool.h>
#ifdef __cplusplus
}
#endif
#ifdef __cplusplus
class pid
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
    float output = 0.0f;
    float integral_limit = 0.0f;
    float output_limit = 0.0f;
    float deadzone = 0.0f;
    float deadzone_compensation = 0.0f; // 死区补偿
    float integral_separation_threshold = 0.0f;
    float error = 0.0f;

    float last_output = 0.0f;

    pid(float kp, float ki, float kd, float integral_limit, float output_limit, float deadzone, float integral_separation_threshold);

    void PID_SetParameters(float kp, float ki, float kd);

    float PID_Compute(float input);

    float PID_ComputeError(float error_); // 可以直接传入误差来进行计算
};

#endif
#endif
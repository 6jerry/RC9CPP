#include "pid.h"

pid::pid(float kp_, float ki_, float kd_, float integral_limit_, float output_limit_, float deadzone_, float integral_separation_threshold_) : kp(kp_), ki(ki_), kd(kd_), integral_limit(integral_limit_), output_limit(output_limit_), deadzone(deadzone_), integral_separation_threshold(integral_separation_threshold_)
{
}

void pid::PID_SetParameters(float kp_, float ki_, float kd_)
{
    kp = kp_;
    ki = ki_;
    kd = kd_;
}

float pid::PID_Compute(float input)
{
    error = setpoint - input;

    // 死区处理
    if (error < deadzone && error > 0)
    {
        error = 0.0f;
    }
    if (error > -deadzone && error < 0)
    {
        error = 0.0f;
    }
    p_out = kp * error;

    error_sum += error;

    // 积分分离
    if (error > integral_separation_threshold)
    {
        error_sum = 0.0f;
    }
    if (error < -integral_separation_threshold)
    {
        error_sum = 0.0f;
    }
    // 积分限幅
    if (error_sum > integral_limit)
    {
        error_sum = integral_limit;
    }
    if (error_sum < -integral_limit)
    {
        error_sum = -integral_limit;
    }
    i_out = ki * error_sum;

    d_out = kd * (error - previous_error);
    previous_error = error;

    output = p_out + i_out + d_out;

    // 输出限幅
    if (output > output_limit)
    {
        output = output_limit;
    }
    else if (output < -output_limit)
    {
        output = -output_limit;
    }

    return output;
}
float pid::PID_ComputeError(float error_)
{
    error = error_;

    // 死区处理
    if (error < deadzone && error > 0)
    {
        error = 0.0f;
    }
    if (error > -deadzone && error < 0)
    {
        error = 0.0f;
    }
    p_out = kp * error;

    error_sum += error;

    // 积分分离
    if (error > integral_separation_threshold)
    {
        error_sum = 0.0f;
    }
    if (error < -integral_separation_threshold)
    {
        error_sum = 0.0f;
    }
    // 积分限幅
    if (error_sum > integral_limit)
    {
        error_sum = integral_limit;
    }
    if (error_sum < -integral_limit)
    {
        error_sum = -integral_limit;
    }
    i_out = ki * error_sum;

    d_out = kd * (error - previous_error);
    previous_error = error;

    output = p_out + i_out + d_out;

    // 输出限幅
    if (output > output_limit)
    {
        output = output_limit;
    }
    else if (output < -output_limit)
    {
        output = -output_limit;
    }

    return output;
}
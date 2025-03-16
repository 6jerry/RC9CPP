/*******************************************************************************
 * @file pid.cpp
 * @author 6Jerry (1517752988@qq.com)
 * @brief pid controller.
 * @version 1.0
 * @date 2024-10-26
 *
 * @copyright Copyright (c) 2024-10-26 6Jerry
 *
 * @license MIT
 *
 * @disclaimer This software is provided "as is", without warranty of any kind, express or implied,
 *             including but not limited to the warranties of merchantability, fitness for a
 *             particular purpose and noninfringement. In no event shall the authors be liable for any
 *             claim, damages or other liability, whether in an action of contract, tort or otherwise,
 *             arising from, out of or in connection with the software or the use or other dealings
 *             in the software.
 ******************************************************************************/
#include "pid.h"
pid::pid(float kp_, float ki_, float kd_, float integral_limit_, float output_limit_, float deadzone_, float integral_separation_threshold_) : kp(kp_), ki(ki_), kd(kd_), integral_limit(integral_limit_), output_limit(output_limit_), deadzone(deadzone_), integral_separation_threshold(integral_separation_threshold_), d_filter(1.0f)
{
}

void pid::PID_SetParameters(float kp_, float ki_, float kd_)
{
    kp = kp_;
    ki = ki_;
    kd = kd_;
}
void pid::ConfigAll(float kp_, float ki_, float kd_, float integral_limit_, float output_limit_, float deadzone_, float integral_separation_threshold_, float filter_a)
{
    kp = kp_;
    ki = ki_;
    kd = kd_;
    integral_limit = integral_limit_;
    output_limit = output_limit_;
    deadzone = deadzone_;
    integral_separation_threshold = integral_separation_threshold_;
    d_filter.setAlpha(filter_a);
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

    if (ki == 0.0f)
    {
        error_sum = 0.0f;
    }

    i_out = ki * error_sum;

    d_out = kd * (error - previous_error);
    previous_error = error;

    output = p_out + i_out + d_filter.update(d_out);

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

    i_out = ki * error_sum;

    d_out = kd * (error - previous_error);
    previous_error = error;

    output = p_out + i_out + d_filter.update(d_out);

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
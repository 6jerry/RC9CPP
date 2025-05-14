#include "superpid.h"

superpid::superpid(float kp_, float ki_, float kd_, float output_limit_, float deadzone_, float integral_separation_threshold_) : kp(kp_), ki(ki_), kd(kd_), output_limit(output_limit_),
                                                                                                                                  deadzone(deadzone_),
                                                                                                                                  integral_separation_threshold(integral_separation_threshold_)
{
}
void superpid::config_all(float kp_, float ki_, float kd_, float output_limit_, float deadzone_, float integral_separation_threshold_)
{
    kp = kp_;
    ki = ki_;
    kd = kd_;
    output_limit = output_limit_;
    deadzone = deadzone_;
    integral_separation_threshold = integral_separation_threshold_;
}

void superpid::superPID_SetParameters(float kp_, float ki_, float kd_)
{
    kp = kp_;
    ki = ki_;
    kd = kd_;
}

float superpid::superPID_Compute(float input)
{
    uint32_t current_time = HAL_GetTick(); // 获取当前时间，单位ms
    if (previous_time != 0)
    { // 确保上一次时间不为0
        sampling_period = float(current_time - previous_time) / 1000.0f;
    }
    previous_time = current_time;
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

    error_sum += (error + previous_error) * sampling_period / 2.0f; // 梯形积分

    // 积分分离
    if (error > integral_separation_threshold)
    {
        error_sum = 0.0f;
    }
    if (error < -integral_separation_threshold)
    {
        error_sum = 0.0f;
    }
    if (max_output)
    {
        error_sum = 0.0f; // 积分抗饱和
    }

    if (ki == 0.0f)
    {
        error_sum = 0.0f;
    }

    i_out = ki * error_sum;

    d_out = -kd * (input - previous_input) / sampling_period;

    // 更新状态
    previous_error = error;
    previous_input = input;

    // 输出计算，包括死区补偿
    output = p_out + i_out + d_out;

    // 输出限幅
    if (output >= output_limit)
    {
        output = output_limit;
        max_output = true;
    }
    else if (output <= -output_limit)
    {
        output = -output_limit;
        max_output = true;
    }
    else
    {
        max_output = false;
    }

    return output;
}

float superpid::superPID_ComputeError(float error_, float C_V)
{
    uint32_t current_time = HAL_GetTick(); // 获取当前时间，单位ms
    if (previous_time != 0)
    { // 确保上一次时间不为0
        sampling_period = float(current_time - previous_time) / 1000.0f;
    }
    previous_time = current_time;
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

    error_sum += (error + previous_error) * sampling_period / 2.0f; // 梯形积分

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

    d_out = -kd * (C_V - previous_input) / sampling_period;

    // 更新状态
    previous_error = error;
    previous_input = C_V;

    // 输出计算，包括死区补偿
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

IncrePID::IncrePID(float kp_, float ki_, float kd_, float r_, float output_limit_, float deadzone_) : kp(kp_), kd(kd_), ki(ki_), output_limit(output_limit_), deadzone(deadzone_), r(r_)
{
}

void IncrePID::increPID_SetParameters(float kp_, float ki_, float kd_, float r_)
{
    kp = kp_;
    ki = ki_;
    kd = kd_;
    r = r_;
}

void IncrePID::calc()
{
    // 死区处理
    if (error < deadzone && error > 0)
    {
        error = 0.0f;
    }
    if (error > -deadzone && error < 0)
    {
        error = 0.0f;
    }

    if (if_first_flag)
    {
        last_error = error;
        lalast_error = error;
        if_first_flag = false;
    }

    p_out = kp * (error - last_error);
    i_out = ki * error;
    d_out = kd * (error - 2.0f * last_error + lalast_error);

    output = last_output + p_out + i_out + d_out;

    // 输出限幅
    if (output > output_limit)
    {
        output = output_limit;
    }
    else if (output < -output_limit)
    {
        output = -output_limit;
    }

    lalast_error = last_error;
    last_error = error;
    last_output = output;
}

float IncrePID::increPID_Compute(float input)
{
    // TD();
    // setpoint = V1; // 跟踪微分器输出平滑的期望

    error = setpoint - input;
    calc();

    return output;
}
float IncrePID::increPID_Computerror(float error_) // 直接传入误差
{
    error = error_; // 这个函数没有TD哦，所以用这个的话参数r是没有意义的
    calc();
    return output;
}
void IncrePID::TD()
{
    uint32_t current_time = HAL_GetTick(); // 获取当前时间，单位ms
    if (previous_time != 0)
    { // 确保上一次时间不为0
        Ts = float(current_time - previous_time) / 1000.0f;
    }
    previous_time = current_time;
    fh = -r * r * (V1 - expect) - 2.0f * r * V2;

    V1 += V2 * Ts;
    V2 += fh * Ts;
}

void IncrePID::increPID_setarget(float target_)
{
    setpoint = target_;
}

void IncrePID::config_all(float kp_, float ki_, float kd_, float r_, float output_limit_, float deadzone_)
{
    kp = kp_;
    ki = ki_;
    kd = kd_;
    r = r_;
    output_limit = output_limit_;
    deadzone = deadzone_;
}
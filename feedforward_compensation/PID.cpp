
/*
 * @author loopgad
 * @contact 3280646246@qq.com
 * @license MIT License
 *
 * Copyright (c) 2024 loopgad
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

/*******************************************************************************
 * @file pid.cpp
 * @author 6Jerry (1517752988@qq.com)
 * @modified by loopgad (3280646246@qq.com) [2025-2-28]
 * @brief PID控制器核心实现
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
#include "stm32f4xx_hal.h"


pid::pid(float kp_, float ki_, float kd_, float integral_limit_, float output_limit_, float deadzone_, float integral_separation_threshold_) : kp(kp_), ki(ki_), kd(kd_), integral_limit(integral_limit_), output_limit(output_limit_), deadzone(deadzone_), integral_separation_threshold(integral_separation_threshold_)
{
}

void pid::PID_SetParameters(float kp_, float ki_, float kd_)
{
    kp = kp_;
    ki = ki_;
    kd = kd_;
}


float pid::FeedForwardError(float error_) {
    static uint32_t last_time = 0;
    uint32_t current_time = HAL_GetTick();
    uint32_t delta_time = (current_time - last_time) / 1000;
    //计算前馈量
    if (L != 0) {
        return m * error_ / delta_time -g * (h / L) * error_ / delta_time;
    }
    last_time = current_time;
    return 0.0f;
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

    output = p_out + i_out + d_out + FeedForwardError(error);

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
    FeedForwardError(error);
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

    output = p_out + i_out + d_out + FeedForwardError(error);

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
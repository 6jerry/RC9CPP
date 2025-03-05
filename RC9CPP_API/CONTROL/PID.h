/*******************************************************************************
 * @file pid.h
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
#ifndef PID_H
#define PID_H

#ifdef __cplusplus
extern "C"
{
#endif
#include <stdbool.h>
#include "filters.h"
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

    pid(float kp = 0.0f, float ki = 0.0f, float kd = 0.0f, float integral_limit = 0.0f, float output_limit = 0.0f, float deadzone = 0.0f, float integral_separation_threshold = 0.0f);

    void PID_SetParameters(float kp_, float ki_, float kd_);
    void ConfigAll(float kp_, float ki_, float kd_, float integral_limit_, float output_limit_, float deadzone_, float integral_separation_threshold_, float filter_a = 1.0f);

    float PID_Compute(float input);

    float PID_ComputeError(float error_); // 可以直接传入误差来进行计算

private:
    SimpleLowPassFilter d_filter;
};

#endif
#endif

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
 * @file pid.h
 * @author 6Jerry (1517752988@qq.com)
 * @modified by loopgad (3280646246@qq.com) [2025-2-28]
 * @brief PID控制器，支持重心转移前馈补偿（完全封装版）
 * @version 2.0
 * @date 2024-10-26
 ******************************************************************************/
#ifndef PID_H
#define PID_H

#ifdef __cplusplus
extern "C" {
#endif
#include <stdbool.h>
#include <cstdint>
#ifdef __cplusplus
}
#endif

#ifdef __cplusplus


class pid {
   private:
    //------------------------- 用户可配置参数 -------------------------
   
    const float h = 0.266f;   // 重心高度（单位：米）
    const float L = 0.38755f;    // 轴距（单位：米）
    const float g = 9.81f;   // 重力加速度（默认9.81 m/s²）
    const float m = 19.44f;  // 质量（单位：kg）

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
    float FeedForwardError(float error_); //单独计算前馈
    float PID_ComputeError(float error_); // 可以直接传入误差来进行计算
};

#endif
#endif
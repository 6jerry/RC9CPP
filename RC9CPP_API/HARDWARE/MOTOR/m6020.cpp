/*******************************************************************************
 * @file M6020.cpp
 * @author 6Jerry (1517752988@qq.com)
 * @brief m6020 motor type.
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
#include "m6020.h"
m6020s::m6020s(uint8_t can_id, CAN_HandleTypeDef *hcan_, bool if_double_control_, float kp_r, float ki_r, float kd_r, float r_r_, float kp_p, float ki_p, float kd_p) : CanDevice(M6020, hcan_, can_id), rpm_pid(kp_r, ki_r, kd_r, r_r_, 25000.0f, 1.0f), pos_pid(kp_p, ki_p, kd_p, 10000.0f, 300.0f, 0.01f, 60.0f), dji_motor(3000.0f, 16384, 8191), if_double_control(if_double_control_)
{
}

int16_t m6020s::motor_process()
{

    target_angle_tf();

    real_angle = convert_angle_to_signed(rangle);
    if (real_angle * target_angle >= 0)
    {
        angle_error = target_angle - real_angle;
    }
    else
    {
        if (real_angle > 0 && target_angle < 0)
        {
            float positive = 180.0f - real_angle + 180.0f + target_angle; // 正路径
            float negative = target_angle - real_angle;
            if (abs(positive) <= abs(negative))
            {
                angle_error = positive;
            }
            else
            {
                angle_error = negative; // 选择一个较短的路径
            }
        }
        else if (real_angle < 0 && target_angle > 0)
        {
            float positive = target_angle - real_angle;
            float negative = -(360.0f + real_angle - target_angle);
            if (abs(positive) <= abs(negative))
            {
                angle_error = positive;
            }
            else
            {
                angle_error = negative; // 选择一个较短的路径
            }
        }
    }

    if (if_double_control) // 对于那些可以360度旋转的机构采用双环控制
    {
        target_rpm = pos_pid.PID_ComputeError(angle_error);
        // rpm_pid.setpoint = target_rpm * (float)gear_ratio;
        rpm_pid.increPID_setarget(target_rpm);
        target_v = (int16_t)rpm_pid.increPID_Compute(rpm);
    }
    else
    { // 丸辣，机械已经装完了，只能单环硬调了
      // rpm_pid.integral_separation_threshold = 30.0f;
      // target_v = (int16_t)rpm_pid.PID_ComputeError(angle_error);
    }

    return target_v;
}

void m6020s::can_update(uint8_t can_RxData[8])
{
    uint16_t vangle = (can_RxData[0] << 8) | can_RxData[1];
    rangle = vangle_to_rangle(vangle);

    rpm = (can_RxData[2] << 8) | can_RxData[3];

    int16_t vcurrent = (can_RxData[4] << 8) | can_RxData[5];
    rcurrent = vcurrent_to_rcurrent(vcurrent);
}

void m6020s::target_angle_tf() // 考虑了机械初始安装角度的角度变换
{
    delta_angle = target_relative_angle + init_angle;
    if (delta_angle >= 180.0f)
    {
        delta_angle -= 360.0f;
    }
    else if (delta_angle < -180.0f)
    {
        delta_angle += 360.0f;
    }

    target_angle = delta_angle;
}
void m6020s::set_init_angle(float init_angle_)
{
    init_angle = init_angle_;
}

float m6020s::get_rpm()
{
    return rpm;
}

float m6020s::get_pos()
{
    return real_angle;
}
void m6020s::set_pos(float pos)
{
    target_relative_angle = pos;
}
void m6020s::set_rpm(float power_motor_rpm)
{
    // target_angle = power_motor_rpm;
}
float m6020s::get_relative_pos()
{
    return 0;
}
float m6020s::get_absolute_pos()
{
    return real_angle;
}
void m6020s::set_relative_pos(float relative_pos_)
{
}
void m6020s::set_absolute_pos_multi(float absolute_pos_multi_)
{
}
void m6020s::set_absolute_pos_single(float absolute_pos_single_)
{
    target_angle = absolute_pos_single_;
}
void m6020s::relocate(float new_zero_point)
{
}

// 将角度从 [0, 360] 转换到 [-180, 180]
float m6020s::convert_angle_to_signed(float current_angle)
{
    // 转换角度到 [-180, 180] 区间
    float converted_angle = fmodf(current_angle + 180.0f, 360.0f) - 180.0f;

    // 如果角度为 -180.0，调整为 180.0 以保持一致性
    if (converted_angle == -180.0f)
    {
        converted_angle = 180.0f;
    }

    return converted_angle;
}
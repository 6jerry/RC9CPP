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
m6020s::m6020s(uint8_t can_id, CAN_HandleTypeDef *hcan_) : CanDevice(M6020, hcan_, can_id), dji_motor(3000.0f, 16384, 8191)
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

    /*if (if_double_control) // 对于那些可以360度旋转的机构采用双环控制
    {
        target_rpm = pos_pid.PID_ComputeError(angle_error);
        // target_rpm = 0.0f;
        //  rpm_pid.setpoint = target_rpm * (float)gear_ratio;
        rpm_pid.setpoint = target_rpm * (float)gear_ratio;
        target_v = (int16_t)rpm_pid.PID_Compute(rpm);
    }
    else
    {
        // rpm_pid.integral_separation_threshold = 30.0f;
        // target_v = (int16_t)rpm_pid.PID_ComputeError(angle_error);
    }*/

    // return target_v;

    switch (work_mode)
    {
    case m6020_servo_pid:
        target_v = servo_pid();
        break;
    case m6020_servo_speed_plan:
        target_v = servo_speed_plan();
        break;
    case m6020_rpm_pid:
        target_v = rpm_ctrl();

        break;
    case m6020_F:
        target_v = F_ctrl();
        break;
    }
    // target_test_v = (float)target_v;
    real_F = get_F();
    return target_v;
}

int16_t m6020s::F_ctrl()
{
    target_test_v = (target_F / TorqueConstant) * 1000.0f;
    if (ebable_debug)
    {
        float to_send_datas[2] = {0.0f, (float)rpm};
        sendFloatData(1, to_send_datas, 2);
    }

    return rcurrent_to_vcurrent((target_F / TorqueConstant) * 1000.0f);
}

int16_t m6020s::
    servo_pid()
{
    target_rpm = pos_pid.PID_ComputeError(angle_error);
    if (ebable_debug)
    {
        float to_send_datas[2] = {0.0f, angle_error};
        sendFloatData(1, to_send_datas, 2);
    }

    return rpm_ctrl();
}

int16_t m6020s::
    servo_speed_plan()
{
    return 0;
}

int16_t m6020s::rpm_ctrl()
{
    rpm_pid.setpoint = target_rpm * (float)gear_ratio;

    if (ebable_debug)
    {
        float to_send_datas[2] = {target_rpm, (float)rpm};
        sendFloatData(1, to_send_datas, 2);
    }

    return rcurrent_to_vcurrent(rpm_pid.superPID_Compute(rpm));
}
void m6020s::start_debug()
{
    ebable_debug = true;
}
void m6020s::set_F(float F_)
{
    target_F = F_;
    work_mode = m6020_F;
}

float m6020s::get_F()
{
    return (rcurrent / 1000.0f) * TorqueConstant;
}
void m6020s::config_rpm_pid(float kp, float ki, float kd, float output_limit, float deadzone, float integral_separation_threshold)
{
    rpm_pid.config_all(kp, ki, kd, output_limit, deadzone, integral_separation_threshold);
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
    work_mode = m6020_servo_pid;
    target_relative_angle = pos;
}
void m6020s::set_rpm(float power_motor_rpm)
{
    work_mode = m6020_rpm_pid;
    target_rpm = power_motor_rpm;
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
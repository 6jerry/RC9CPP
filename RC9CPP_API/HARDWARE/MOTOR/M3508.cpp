/*******************************************************************************
 * @file M3508.cpp
 * @author 6Jerry (1517752988@qq.com)
 * @brief m3508 motor type.
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
#include "M3508.h"
m3508p::m3508p(uint8_t can_id, CAN_HandleTypeDef *hcan_, motor_mode mode_, float gear_ratio, float kp_, float ki_, float kd_, float r_) : CanDevice(M3508, hcan_, can_id), gear_ratio(gear_ratio), dji_motor(20000.0f, 16384, 8191), rpm_control(kp_, ki_, kd_, r_, 20000.0f, 5.0f) // 选择使用积分分离的话积分限幅就是无意义的，随便给个爆大的值就行
{
    mode = mode_;
}

int16_t m3508p::motor_process()
{
    // T_TO_C();

    rpm_control.increPID_setarget(target_rpm * gear_ratio);
    vtarget_current = rcurrent_to_vcurrent(rpm_control.increPID_Compute(rpm)) + rcurrent_to_vcurrent(Cff);

    switch (mode)
    {
    case speed:
        /* code */
        break;
    case pos_single:
        /* code */
        break;
    case pos_many:
        /* code */
        break;
    }

    return vtarget_current;
}
void m3508p::can_update(uint8_t can_RxData[8])
{
    uint16_t vangle = (can_RxData[0] << 8) | can_RxData[1];
    rangle = vangle_to_rangle(vangle);

    rpm = (can_RxData[2] << 8) | can_RxData[3];

    int16_t vcurrent = (can_RxData[4] << 8) | can_RxData[5];
    rcurrent = vcurrent_to_rcurrent(vcurrent); // 虚拟电流，mA

    if (mode == pos_many)
    {
        many_pos_locate();
    }
}
float m3508p::get_rpm()
{
    return (float)rpm / gear_ratio;
}

void m3508p::set_rpm(float power_motor_rpm)
{
    target_rpm = power_motor_rpm;
}

void m3508p::T_TO_C()
{
    Cff = (Tff / gear_ratio) * 64020.49;
}
void m3508p::set_fTff(float Tff_)
{
    Tff = Tff_;
}

void m3508p::many_pos_locate() // 差分定位计算3508多圈位置
{
    if (if_init)
    {
        init_cnt++;
        if (init_cnt > 6)
        {
            locate_restart();
            if_init = false;
        }
    }

    last_pos = now_pos;

    now_pos = rangle;

    temp_delta = now_pos - last_pos;

    if (now_pos > 357.0f)
    {
        if (last_pos < 3.0f)
        {
            delta_pos = -(360.0f - now_pos + last_pos);
        }
        else
        {
            delta_pos = temp_delta;
        }
    }
    else if (now_pos < 3.0f)
    {
        if (last_pos > 357.0f)
        {
            delta_pos = now_pos + (360.0f - last_pos);
        }
        else
        {
            delta_pos = temp_delta;
        }
    }
    else
    {
        delta_pos = temp_delta;
    }

    pos_sum += delta_pos;
}

void m3508p::locate_restart()
{
    pos_sum = 0.0f;
}
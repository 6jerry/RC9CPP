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
m3508p::m3508p(uint8_t can_id, CAN_HandleTypeDef *hcan_, uint8_t gear_ratio, float kp_, float ki_, float kd_) : CanDevice(M3508, hcan_, can_id), gear_ratio(gear_ratio), pid(kp_, ki_, kd_, 1000000.0f, 20000.0f, 5.0f, 960.0f), dji_motor(20000.0f, 16384, 8191) // 选择使用积分分离的话积分限幅就是无意义的，随便给个爆大的值就行
{
}

int16_t m3508p::motor_process()
{
    setpoint = target_rpm * (float)gear_ratio;
    vtarget_current = rcurrent_to_vcurrent(PID_Compute(rpm));
    return vtarget_current;
}
void m3508p::can_update(uint8_t can_RxData[8])
{
    uint16_t vangle = (can_RxData[0] << 8) | can_RxData[1];
    rangle = vangle_to_rangle(vangle);

    rpm = (can_RxData[2] << 8) | can_RxData[3];

    int16_t vcurrent = (can_RxData[4] << 8) | can_RxData[5];
    rcurrent = vcurrent_to_rcurrent(vcurrent);
}
float m3508p::get_rpm()
{
    return rpm / 19.0f;
}

void m3508p::set_rpm(float power_motor_rpm)
{
    target_rpm = power_motor_rpm;
}
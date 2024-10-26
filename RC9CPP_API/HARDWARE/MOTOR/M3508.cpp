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
    return rpm;
}

void m3508p::set_rpm(float power_motor_rpm)
{
    target_rpm = power_motor_rpm;
}
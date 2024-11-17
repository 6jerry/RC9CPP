#ifndef MOTOR_H
#define MOTOR_H

#ifdef __cplusplus
extern "C"
{
#endif
#include "can_device.h"
#ifdef __cplusplus
}
#endif

// 通用电机接口，便于底盘和各类机构调用的，总体来说分为动力电机和伺服电机,使用位置控制的m3508也属于伺服电机类
#ifdef __cplusplus

enum motor_mode
{
    speed,
    pos_pid,
    pos_speedplan,
    standby
};
class power_motor
{

public:
    motor_mode mode = standby;
    virtual float get_rpm() = 0;
    virtual void set_rpm(float power_motor_rpm) = 0; // 获取当前转速和设置目标转速的通用接口
    void switch_mode(motor_mode target_mode);

    virtual void set_rpm_ff(float power_motor_rpm, float ff) {}; // 设置速度和前馈值
};

class dji_motor
{
private:
public:
    dji_motor(float max_rcurrent_, int16_t max_vcurrent_, uint16_t max_vangle_);

    float rangle = 0;
    int16_t rpm = 0;
    float rcurrent = 0;
    int16_t vtarget_current = 0;

    float max_rcurrent = 0.0f;
    int16_t max_vcurrent = 0;
    uint16_t max_vangle = 0;

    float vcurrent_to_rcurrent(int16_t vc);
    int16_t rcurrent_to_vcurrent(float rc);
    float vangle_to_rangle(uint32_t va);
};

#endif

#endif

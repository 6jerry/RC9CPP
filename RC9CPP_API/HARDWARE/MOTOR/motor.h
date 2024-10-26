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
class power_motor
{

public:
    virtual float get_rpm() = 0;
    virtual void set_rpm(float power_motor_rpm) = 0; // 获取当前转速和设置目标转速的通用接口
};


class servo_motor
{
public:
    virtual float get_relative_pos() = 0;
    virtual float get_absolute_pos() = 0;
    virtual void set_relative_pos(float relative_pos_) = 0;               // 设置相对位置
    virtual void set_absolute_pos_multi(float absolute_pos_multi_) = 0;   // 绝对位置（多圈）
    virtual void set_absolute_pos_single(float absolute_pos_single_) = 0; // 绝对位置（单圈）
    virtual void relocate(float new_zero_point) = 0;                      // 重定位相对位置零点
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

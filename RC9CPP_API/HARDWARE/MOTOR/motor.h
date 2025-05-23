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
    pos_single,
    pos_many,
    standby,

};
class power_motor
{

public:
    motor_mode mode = speed;
    virtual float get_rpm() = 0;
    virtual void set_rpm(float power_motor_rpm) = 0; // 获取当前转速和设置目标转速的通用接口
    void switch_mode(motor_mode target_mode);

    virtual void send_rpm(float power_motor_rpm) {};

    virtual void set_ff_current(float target_c_) {};

    virtual float get_pos() {};
    virtual void set_pos(float pos) {}; // 获取当前位置和设置目标位置的通用接口

    virtual void set_current(float target_c_) {};
    virtual void set_dis(float dis) {}; // 设置距离

    virtual void relocate_dis(float dis) {}; // 重新定位距离

    virtual float get_dis() {}; // 获取距离

    virtual bool set_dis_speedplan(float targetdis, float max_speed, float max_acc, float max_dec, float finalspeed) {}; // 设置距离和速度和加速度
    virtual bool set_pos_speedplan(float targetpos, float max_speed, float max_acc, float max_dec, float finalspeed) {};

    virtual void dis_speedplan_restart() {}; // 重新开始距离速度规划
    virtual void pos_speedplan_restart() {};

    virtual void set_angle(float angle) {}; // 设置角度

    virtual void relocate_pos(float angle) {}; // 重新定位角度

    virtual float get_odom() {}; // 获取里程

    virtual void set_F(float F_) {}; // 设置力矩

    virtual float get_F() {}; // 获取力矩

    virtual void set_rpm_ff(float power_motor_rpm, float ff) {}; // 设置速度和前馈值
};

class dji_motor
{
private:
public:
    dji_motor(float max_rcurrent_, int16_t max_vcurrent_, uint16_t max_vangle_);

    float rangle = 0;
    int16_t rpm = 0.0f;
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

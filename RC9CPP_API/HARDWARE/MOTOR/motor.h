#ifndef MOTOR_H
#define MOTOR_H

#ifdef __cplusplus
extern "C"
{
#endif
#include "fdcan_device.h"
#include "TaskManager.h"
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

#define dji_id_1 0x201
#define dji_id_2 0x202
#define dji_id_3 0x203
#define dji_id_4 0x204

#define dji_id_5 0x205
#define dji_id_6 0x206
#define dji_id_7 0x207
#define dji_id_8 0x208

#define dji_id_9 0x209
#define dji_id_10 0x20A
#define dji_id_11 0x20B

#define vesc_id_0 0x900
#define vesc_id_1 0x901
#define vesc_id_2 0x902
#define vesc_id_3 0x903
#define vesc_id_4 0x904
#define vesc_id_5 0x905
#define vesc_id_6 0x906
#define vesc_id_7 0x907
#define vesc_id_8 0x908

enum DjiMotorType
{
    M3508_M2006,

    M6020

};

#define MAX_INSTANCES 8
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
    dji_motor(float max_rcurrent_, int16_t max_vcurrent_, uint16_t max_vangle_, DjiMotorType type_, uint32_t can_id_, FDCAN_HandleTypeDef *hcan_);

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

    virtual int16_t motor_process() = 0;

    DjiMotorType type;

public:
    static dji_motor *m3508_instances_can1[MAX_INSTANCES];
    static dji_motor *m3508_instances_can2[MAX_INSTANCES];
    static dji_motor *m3508_instances_can3[MAX_INSTANCES];
    static uint8_t instanceCount_m3508_can1;
    static uint8_t instanceCount_m3508_can2;
    static uint8_t instanceCount_m3508_can3;

    static dji_motor *m6020_instances_can1[MAX_INSTANCES];
    static dji_motor *m6020_instances_can2[MAX_INSTANCES];
    static dji_motor *m6020_instances_can3[MAX_INSTANCES];
    static uint8_t instanceCount_m6020_can1;
    static uint8_t instanceCount_m6020_can2;
    static uint8_t instanceCount_m6020_can3;
};

class dji_motor_handle : public ITaskProcessor
{
public:
    void process_data();
};

#endif

#endif

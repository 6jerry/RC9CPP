#ifndef DEBUG_XBOX_H
#define DEBUG_XBOX_H

#ifdef __cplusplus
extern "C"
{
#endif
#include "xbox.h"
#include "motor.h"
#include "TaskManager.h"
#include <arm_math.h>
#include "RC9Protocol.h"
#include "robot_chassis.h"
#include "GCFSM.h"
#include "TrapezoidalPlanner.h"
#ifdef __cplusplus
}
#endif
#ifdef __cplusplus

class debug_xbox : public xbox, public ITaskProcessor
{
public:
    float map_value = 0.0f;
    uint8_t if_start = 0;

public:
    void process_data();
    void btn_scan();
    void btnconfig_init();
    debug_xbox();
};

class algorithm_debug : public RC9subscriber
{

public:
    float get_input_mapvalue();
    uint8_t get_input_start();
    void pid_send_debuginfo(float target, float now_value);
    void filter_send_debuginfo(float orin_data, float filted_data);
    void speedplan_send_debuginfo(float target_speed, float now_speed);

    void addxbox(debug_xbox *debug_);
    void add_IO(debug_xbox *debug_, RC9Protocol *port_);

    float temp_param[6] = {0.0f}; // 暂存上位机传过来的参数
    void DataReceivedCallback(const uint8_t *byteData, const float *floatData, uint8_t id, uint16_t byteCount) override;

private:
    debug_xbox *debug;
};

class moters_debug_xbox : public xbox, public ITaskProcessor
{
public:
    power_motor *debug_motor = nullptr;

    uint8_t debug_mode = 1, start_flag = 0;

    float max_F = 2.2f, max_rpm = 140.0f, setted_f = 0.0f;

    float max_speed = 500.0f, max_acc = 800.0f, max_dec = 800.0f, final_speed = 0.0f, target_dis = 600.0f;

public:
    void process_data();
    void btn_scan();
    void btnconfig_init();
    void add_motor(power_motor *motor_);
    moters_debug_xbox();
};

class xbox_debug_base : public xbox, public ITaskProcessor
{
public:
    uint8_t mode_flag = 2, start_flag = 0, lb_flag = 0, rb_flag = 0;

    virtual void not_start() {};
    virtual void mode_0() {};
    virtual void mode_1() {};
    virtual void mode_2() {};
    virtual void mode_3() {};
    virtual void mode_4() {};

    virtual void lb_on() {};
    virtual void rb_on() {};
    virtual void lb_off() {};
    virtual void rb_off() {};

    void process_data();
    void btn_scan();
    void btnconfig_init();
    xbox_debug_base();
};

class chassis_adjust_xbox : public xbox_debug_base, public chassis_user
{
private:
    Vector2D t_points[2] = {{0.0f, 0.0f}, {0.0f, 0.0f}};

public:
    void not_start() override;
    void mode_2() override;
    void mode_1() override;
    void mode_3() override;
};

#endif
#endif
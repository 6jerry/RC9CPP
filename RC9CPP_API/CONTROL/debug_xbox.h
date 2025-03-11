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

class chassis_debug_xbox : public xbox, public ITaskProcessor, public chassis_user
{
public:
    uint8_t btny_flag = 0, btnx_flag = 0, btna_flag = 0, btnb_flag = 0, btnshare_flag = 0;

    float full_speed = 0.0f, full_w = 0.0f, setted_f = 0.0f;

    uint8_t priocode = 1;

    uint8_t currentState = 0;
    // catcher_fsm *claw = nullptr;

public:
    void
    btn_config();
    void process_data();

    void btn_scan();

    chassis_debug_xbox(float full_speed_, float full_w_);
};

#endif
#endif
#ifndef SHOOT_PID_ADJUST_H
#define SHOOT_PID_ADJUST_H

#ifdef __cplusplus
extern "C"
{
#endif
#include "debug_xbox.h"
#include "PID.h"
#include "RC9Protocol.h"
#include "imu.h"
#include "motor.h"
#include "fdcan_device.h"
#ifdef __cplusplus
}
#endif
#ifdef __cplusplus

class shoot_pid_xbox : public xbox_debug_base
{
public:
    power_motor *shoot_motor = nullptr;
    pid shoot_control;
    RC9subscriber *msg_send = nullptr;
    imu *encoder = nullptr;

    float now_dis = 0.0f, target_dis = 0.05f, max_rpm = 120.0f,max_dis=36.0f;

public:
    void not_start() override;
    void mode_2() override;
    void mode_3() override;
    void xbox_on() override;
};

#endif
#endif

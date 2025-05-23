#ifndef BALL_SHOOTER_H
#define BALL_SHOOTER_H

#ifdef __cplusplus
extern "C"
{
#endif
#include "TaskManager.h"
#include "SuperPID.h"
#include "motor.h"
#include "PID.h"
#include "SuperPID.h"
#include "gpio.h"
#include "imu.h"
#include "TrapezoidalPlanner.h"
#include "xbox.h"
#include "debug_xbox.h"
#ifdef __cplusplus
}
#endif

#ifdef __cplusplus

enum shooter_mode
{
    shooter_current,
    shooter_rpm,
    shooter_dis,
    shooter_stop,
    shooter_custom_rpm,
    shooter_laser_init

};

class BallShooter : public ITaskProcessor, public RC9subscriber
{
private:
    power_motor *pull_moter = nullptr;
    imu *laser = nullptr, *a_encoder = nullptr;

    float min_dis = 0.06f, max_dis = 0.485f, real_dis = 0.0f, target_dis = 0.0f, target_rpm = 0.0f, target_current = 0.0f;

    shooter_mode workmode = shooter_current;

public:
    IncrePID pull_dis_control;
    void process_data();
    void set_pull_dis(float dis);

    void add_moter(power_motor *moter_);
    void add_laser(imu *laser_);

    void send_current(float current);
    void send_rpm(float rpm);
    void set_motor_rpm(float rpm);
    void set_ff(float ff_);

    void stop();
    float get_pull_dis();
};

class ball_shooter_xbox : public xbox, public ITaskProcessor
{
private:
    const float max_rpm = 1000.0f;
    float K = 0.0f, ff_current = 0.0f;
    float R = 0.0f, max_dis = 0.42f;

    BallShooter *shooter = nullptr;
    uint8_t debug_mode = 1, start_flag = 0;

public:
    void process_data();
    void btn_scan();
    void btnconfig_init();
    ball_shooter_xbox();

    void add_shooter(BallShooter *shooter_);
};

#endif
#endif
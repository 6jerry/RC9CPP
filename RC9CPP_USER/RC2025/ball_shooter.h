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

#include "debug_xbox.h"
#ifdef __cplusplus
}
#endif

#ifdef __cplusplus

class BallShooter : public ITaskProcessor, public algorithm_debug
{
private:
    power_motor *pull_moter = nullptr;
    imu *laser = nullptr;

    float min_dis = 0.06f, max_dis = 0.40f, real_dis = 0.0f, target_dis = 0.0f, max_rpm = 200.0f;

public:
    IncrePID pull_dis_control;
    void process_data();
    void set_pull_dis(float dis);

    void add_moter(power_motor *moter_);
    void add_laser(imu *laser_);
};

#endif
#endif
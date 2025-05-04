#ifndef AUTO_LOCK_H
#define AUTO_LOCK_H

#ifdef __cplusplus
extern "C"
{
#endif
#include "debug_xbox.h"
#include "motor.h"
#include "TaskManager.h"
#include <arm_math.h>
#include "RC9Protocol.h"
#include "robot_chassis.h"
#include "PID.h"
#include "imu.h"
#include "TrapezoidalPlanner.h"
#ifdef __cplusplus
}
#endif
#ifdef __cplusplus

class auto_lock_test : public xbox_debug_base, public chassis_user
{
private:
    Vector2D center_point, tan_dir, nor_dir;                            // 圆心坐标
    float dis_2_center = 0.0f, center_heading = 0.0f, nor_speed = 0.0f; // 半径
    float last_tick = 0.0f;                                            

    float radius[4] = {2.25f, 2.5f, 2.75f, 3.0f}; // 半径

    void calc_error(); // 计算误差
    void Fine_tune(bool up, bool down, bool left, bool right); // 微调

    void mode_2() override;
    void mode_3() override;

    void mode_1() override;
    void xbox_on() override;

    pid nor_control; // 半径控制

    imu *imu_ptr; // 指向imu类的指针

    TrapezoidalPlanner1D planner;

public:
    auto_lock_test(imu *imu_ptr_);
};

#endif
#endif
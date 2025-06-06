#ifndef CHASSIS_DEBUG_H
#define CHASSIS_DEBUG_H

#ifdef __cplusplus
extern "C"
{
#endif
#include "debug_xbox.h"
#include "robot_chassis.h"
#include "RC9Protocol.h"
#include "imu.h"
#ifdef __cplusplus
}
#endif
#ifdef __cplusplus

class chassis_adjust_xbox : public xbox_debug_base, public chassis_user
{
private:
    Vector2D max_target_robot_vel; // 最大目标速度
		float lock_yaw = 0.0f;
    imu *imu_ptr; // 指向imu类的指针
public:
    chassis_adjust_xbox(imu *imu_ptr_);
    void not_start() override;
    void mode_2() override;
    void mode_1() override;
};

#endif
#endif
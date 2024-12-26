#ifndef ODOMETRY_H
#define ODOMETRY_H

#ifdef __cplusplus
extern "C"
{
#endif
#include "imu.h"
#include "motor.h"
#include "TaskManager.h"
#include "Vector2D.h"
#ifdef __cplusplus
}
#endif

#ifdef __cplusplus
enum OdomType
{
    omni3,
    omni3_unusual,
    omni4,
    mecanum4,
    diff_drive
};
class odometry : public ITaskProcessor
{
private:
    power_motor *chassis_motors[4] = {nullptr};
    float last_motor_odom[4] = {0.0f}, delta_motor_odom[4] = {0.0f}; // 左前右前，左后右后
    imu *IMU;
    OdomType odomtype = diff_drive;
    float chassis_l = 0.0f, chassis_w = 0.0f;
    void update_mecanum_odom();

public:
    void process_data();
    float now_heading = 0.0f, now_heading_angle = 0.0f, now_heading_delta = 0.0f;
    Vector2D world_pose, robot_speed, world_speed;
    float get_world_x();
    float get_world_y();

    void odom_restart();

    odometry(power_motor *motor1, power_motor *motor2, power_motor *motor3, power_motor *motor4, imu *IMU_, OdomType odomtype_, float chassis_l_ = 0.0f, float chassis_w_ = 0.0f);
};
#endif

#endif
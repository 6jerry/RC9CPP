#include "chassis_debug.h"

chassis_adjust_xbox::chassis_adjust_xbox(imu *imu_ptr_)
{
    imu_ptr = imu_ptr_;
}

void chassis_adjust_xbox::not_start()
{
    Vector2D target(0.0f, 0.0f);
    set_RobotVel(target, 0);
    set_RobotW(0.0f, 0);
    max_target_robot_vel.x = 3.0f;
    max_target_robot_vel.y = 3.0f;
}

void chassis_adjust_xbox::mode_2()
{
    Vector2D tvel_((max_target_robot_vel.x * xbox_msgs.joyLHori_map), (max_target_robot_vel.y * xbox_msgs.joyLVert_map));
    set_RobotVel(tvel_, 3.0f);
    set_RobotW(-(2.0f * xbox_msgs.joyRHori_map), 0);
		lock_yaw = imu_ptr->get_yaw_rad() * 57.296f;
}

void chassis_adjust_xbox::mode_1()
{
    Vector2D tvel_((max_target_robot_vel.x * xbox_msgs.joyLHori_map), (max_target_robot_vel.y * xbox_msgs.joyLVert_map));
    set_RobotVel(tvel_, 3.0f);
    //set_RobotW(-(2.0f * xbox_msgs.joyRHori_map), 0);

    // 防止舵轮偏移
    if(xbox_msgs.joyRHori_map == 0.0f)
    {
        Correct_yaw(lock_yaw);
    }
    else
    {
        set_RobotW(-(2.0f * xbox_msgs.joyRHori_map), 0);
        lock_yaw = imu_ptr->get_yaw_rad() * 57.296f;
    }
}
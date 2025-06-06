#include "chassis_debug.h"

void chassis_adjust_xbox::not_start()
{
    Vector2D target(0.0f, 0.0f);
    set_RobotVel(target, 0);
    set_RobotW(0.0f, 0);
    max_target_robot_vel.x = 5.0f;
    max_target_robot_vel.y = 5.0f;
}

void chassis_adjust_xbox::mode_2()
{
    Vector2D tvel_((max_target_robot_vel.x * xbox_msgs.joyLHori_map), (max_target_robot_vel.y * xbox_msgs.joyLVert_map));
    set_WorldVel(tvel_, 2.5f);
    set_RobotW(-(2.0f * xbox_msgs.joyRHori_map), 0);
}

void chassis_adjust_xbox::mode_1()
{
    Vector2D tvel_((max_target_robot_vel.x * xbox_msgs.joyLHori_map), (max_target_robot_vel.y * xbox_msgs.joyLVert_map));

    set_WorldVel(tvel_, 0);
    yaw_TurnTo(90.0f * (xbox_msgs.trigLT_map - xbox_msgs.trigRT_map), 0);
}

void chassis_adjust_xbox::mode_3()
{
    if ((xbox_msgs.trigLT_map - xbox_msgs.trigRT_map) == 0.0f)
    {
        yaw_lock();
    }
    else
    {

        set_RobotW(-3.0f * (xbox_msgs.trigLT_map - xbox_msgs.trigRT_map), 0);
    }
    Vector2D tvel_((max_target_robot_vel.x * xbox_msgs.joyLHori_map), (max_target_robot_vel.y * xbox_msgs.joyLVert_map));

    set_WorldVel(tvel_, 0);
}

void chassis_adjust_xbox::xbox_on()
{
    //post->imu_rst();
    post->imu_relocate(0.0f, 0.0f, 0.0f);
}
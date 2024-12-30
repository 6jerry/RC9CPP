#include "odometry.h"

void odometry::process_data()
{
    switch (odomtype)
    {
    case mecanum4:
        update_mecanum_odom();
        break;

    case omni3:
        break;

    case omni3_unusual:
        break;

    case omni4:
        break;

    case diff_drive:
        break;
    }
}

void odometry::update_mecanum_odom()
{

    delta_motor_odom[0] = chassis_motors[0]->get_odom() - last_motor_odom[0];
    delta_motor_odom[1] = chassis_motors[1]->get_odom() - last_motor_odom[1];
    delta_motor_odom[2] = -chassis_motors[2]->get_odom() - last_motor_odom[2];
    delta_motor_odom[3] = -chassis_motors[3]->get_odom() - last_motor_odom[3];

    now_heading_delta = ((-delta_motor_odom[0] - delta_motor_odom[1] + delta_motor_odom[2] + delta_motor_odom[3]) / 0.46986f);
    now_heading += now_heading_delta;
    now_heading_angle = now_heading * 57.296f;
    float delta_robot_pos_x = (delta_motor_odom[0] - delta_motor_odom[1] + delta_motor_odom[2] - delta_motor_odom[3]) / 4.0f;

    float delta_robot_pos_y = (delta_motor_odom[0] + delta_motor_odom[1] + delta_motor_odom[2] + delta_motor_odom[3]) / 4.0f;

    float delta_world_pos_x = delta_robot_pos_x * cos(now_heading) - delta_robot_pos_y * sin(now_heading);

    float delta_world_pos_y = delta_robot_pos_x * sin(now_heading) + delta_robot_pos_y * cos(now_heading);

    world_pose.x += delta_world_pos_x;
    world_pose.y += delta_world_pos_y;

    last_motor_odom[0] = chassis_motors[0]->get_odom();
    last_motor_odom[1] = chassis_motors[1]->get_odom();
    last_motor_odom[2] = -chassis_motors[2]->get_odom();
    last_motor_odom[3] = -chassis_motors[3]->get_odom();
}
odometry::odometry(power_motor *right_front_motor, power_motor *right_back_motor, power_motor *left_back_motor, power_motor *left_front_motor, imu *IMU_, OdomType odomtype_, float chassis_l_, float chassis_w_) : IMU(IMU_), odomtype(odomtype_), chassis_l(chassis_l_), chassis_w(chassis_w_)
{
    chassis_motors[0] = right_front_motor;
    chassis_motors[1] = right_back_motor;
    chassis_motors[2] = left_back_motor;
    chassis_motors[3] = left_front_motor;
}
void odometry::odom_restart()
{
    world_pose.x = 0.0f;
    world_pose.y = 0.0f;
    now_heading = 0.0f;
}

float odometry::get_world_x()
{
    return world_pose.x;
}
float odometry::get_world_y()
{
    return world_pose.y;
}
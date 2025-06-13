#include "chassis_debug.h"

chassis_adjust_xbox::chassis_adjust_xbox(imu *imu_ptr_)
{
    imu_ptr = imu_ptr_;
    center_point.x = 5.788f;
    center_point.y = 0.7505f;

    //    center_point.x = 3.000f;
    //    center_point.y = 14.215f;
}

void chassis_adjust_xbox::calc_error()
{
    Vector2D now_point;
    now_point.x = get_world_x();
    now_point.y = get_world_y();

    Vector2D dis = center_point - now_point;

    nor_dir = dis.normalize();
    tan_dir = Vector2D(nor_dir.y, -nor_dir.x).normalize();

    dis_2_center = dis.magnitude();

    center_heading = -atan2f(nor_dir.x, nor_dir.y) * 57.296f;
    ; // 角度对准圆心
}

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
    set_WorldVel(tvel_, 2.5f); // 以后再改
    set_RobotW(-(2.0f * xbox_msgs.joyRHori_map), 0);

       // auto_shooter->set_shooter_mode(shooter_hand);
    // auto_shooter->shooter_info.hand_shooter_rpm = (xbox_msgs.trigLT_map - xbox_msgs.trigRT_map) * 200.f;
    //		if(test_flag == 1)
    //    {
    //			auto_shooter->encoder->send_reset();
    //		  test_flag = 0;
    //		}
    if (lb_flag)
    {
        auto_yunball_ptr->start_yunball();
        lb_flag = 0;
    }
    if (rb_flag)
    {
        auto_yunball_ptr->control_claw(true);
        auto_yunball_ptr->control_motor(xbox_msgs.joyRVert_map);
    }
    else
    {
        auto_yunball_ptr->control_claw(false);
    }
}

void chassis_adjust_xbox::mode_1()
{

    calc_error();

    Vector2D target(0.0f, 0.0f);
    set_RobotVel(target, 0);

    if (abs(center_heading - get_yaw()) < 0.4f)
    {
        set_RobotW(0.0f, 0);
        auto_shooter->set_auto_byFitter(PID, dis_2_center);
    }
    else
    {
        yaw_TurnTo(center_heading, 0);
    }
}

void chassis_adjust_xbox::mode_3()
{
    // set_WorldVel(Vector2D(0.0f, 0.4f), 0);
    Vector2D tvel_((max_target_robot_vel.x * xbox_msgs.joyLHori_map), (max_target_robot_vel.y * xbox_msgs.joyLVert_map));
    set_WorldVel(tvel_, 2.5f); // 以后再改
    set_RobotW(-(2.0f * xbox_msgs.joyRHori_map), 0);
    if (lb_flag)
    {
        auto_yunball_ptr->start_yunball();
        putball = 1;
        lb_flag = 0;
    }
    if (rb_flag)
    {
        if (auto_yunball_ptr->start_putball())
        {
            rb_flag = 0;
            // mode_flag = 2;
        }
    }
}

void chassis_adjust_xbox::mode_1()
{	  
	calc_error();
    yaw_TurnTo(center_heading, 0);
}

void chassis_adjust_xbox::mode_3()
{
    auto_yunball_ptr->control_motor(xbox_msgs.joyRHori_map);
    if(lb_flag)
    {auto_yunball_ptr->control_lift(true);}
    else
    {auto_yunball_ptr->control_lift(false);}

    if(rb_flag)
    {auto_yunball_ptr->control_claw(true);}
    else
    {auto_yunball_ptr->control_claw(false);}
}

void chassis_adjust_xbox::mode_4()
{
    Vector2D tvel_((max_target_robot_vel.x * xbox_msgs.joyLHori_map), (max_target_robot_vel.y * xbox_msgs.joyLVert_map));
    set_RobotVel(tvel_, 3.0f);
    // set_RobotW(-(2.0f * xbox_msgs.joyRHori_map), 0);

    // 防止舵轮偏移
    if (xbox_msgs.joyRHori_map == 0.0f)
    {
        Correct_yaw(lock_yaw);
    }
    else
    {
        set_RobotW(-(2.0f * xbox_msgs.joyRHori_map), 0);
        lock_yaw = imu_ptr->get_yaw_rad() * 57.296f;
    }
}

void chassis_adjust_xbox::xbox_on()
{
    // post->imu_rst();
    imu_ptr->imu_relocate(0.0f, 0.0f, 0.0f);
    ros_imu->imu_rst();
}

void chassis_adjust_xbox::add_AutoShooter(AutoShooter *auto_shooter_)
{
    auto_shooter = auto_shooter_;
}

void chassis_adjust_xbox::add_autoyunball(auto_yunball *auto_yunball_)
{
    auto_yunball_ptr = auto_yunball_;
}
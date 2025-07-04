#include "chassis_debug.h"

chassis_adjust_xbox::chassis_adjust_xbox(imu *imu_ptr_)
{
    imu_ptr = imu_ptr_;
//    center_point.x = 5.8f;
//    center_point.y = 0.73f;

    center_point.x = 3.000f;
    center_point.y = 14.355f;
}

void chassis_adjust_xbox::calc_error()
{
    Vector2D now_point;
    now_point.x = get_world_x();
    now_point.y = get_world_y();
    Vector2D dis = {0, 0};

    if (cnt_flag == 0)
    {
        dis = center_point - now_point;
        dis_2_center = dis.magnitude();
    }
    else
    {
        dis = robot_point - now_point;
        dis_2_center = dis.magnitude() - pass_correct_distance;
    }

    nor_dir = dis.normalize();

    center_heading = -atan2f(nor_dir.x, nor_dir.y) * 57.296f;
    ; // 角度对准圆心
}

void chassis_adjust_xbox::calc_robopoint()
{
    Vector2D now_point;
    now_point.x = get_world_x();
    now_point.y = get_world_y();
    Vector2D dis = {0, 0};
    dis = robot_point - now_point;

    dis_2_robot = dis.magnitude();
    nor_dir_robot = dis.normalize();
    robot_heading = -atan2f(nor_dir_robot.x, nor_dir_robot.y) * 57.296f;
}

void chassis_adjust_xbox::not_start()
{
    Vector2D target(0.0f, 0.0f);
    set_RobotVel(target, 0);
    set_RobotW(0.0f, 0);
    max_target_robot_vel.x = 5.0f;
    max_target_robot_vel.y = 5.0f;
    auto_yunball_ptr->stop();
}

void chassis_adjust_xbox::mode_2()
{
    calc_error();
    Vector2D tvel_((max_target_robot_vel.x * xbox_msgs.joyLHori_map), (max_target_robot_vel.y * xbox_msgs.joyLVert_map));
    set_WorldVel(tvel_, 2.5f); // 以后再改
    set_RobotW(-(2.0f * xbox_msgs.joyRHori_map), 0);

    if (lb_flag)
    {
        auto_yunball_ptr->start_yunball();
        lb_flag = 0;
    }
    if (rb_flag)
    {
        if (auto_yunball_ptr->start_putball())
        {
            rb_flag = 0;
        }
    }

    if (btn_start_flag)
    {
        reset_swerve();
        btn_start_flag = 0;
    }
}

void chassis_adjust_xbox::mode_1()
{
    calc_error();

    Vector2D target(0.0f, 0.0f);
    set_RobotVel(target, 0);

        if (abs(center_heading - get_yaw()) < limit_yaw_error && abs(get_chassis_yaw_speed()) < limit_yaw_speed)
       {
           set_RobotW(0.0f, 0);
           auto_shooter->set_auto_byFitter(PID, dis_2_center);
            //auto_shooter->set_auto_byDis(PID, debug_dis);
            cnt_flag = 0;
           mode_flag = 2;
       }
       else
       {
           yaw_TurnTo(center_heading, 0);
       }

    //yaw_TurnTo(center_heading, 0);
    // auto_shooter->set_auto_byFitter(PID, dis_2_center);
//    auto_shooter->set_auto_byDis(PID, debug_dis);
//    mode_flag = 2;
}

void chassis_adjust_xbox::mode_3()
{
    auto_yunball_ptr->control_turn_motor(xbox_msgs.joyRHori_map);
		auto_yunball_ptr->control_lift_motor(xbox_msgs.joyLVert_map);

    if (rb_flag)
    {
        auto_yunball_ptr->control_claw(true);
    }
    else
    {
        auto_yunball_ptr->control_claw(false);
    }
}

void chassis_adjust_xbox::mode_4()
{
    Vector2D target(0.0f, 0.0f);
    set_RobotVel(target, 0);

    /*

    calc_robopoint();

    if (abs(robot_heading - get_yaw()) < limit_yaw_error && abs(get_chassis_yaw_speed()) < limit_yaw_speed)
    {
        set_RobotW(0.0f, 0);
        auto_shooter->set_auto_byFitter(PID, dis_2_robot);
        // auto_shooter->set_auto_byDis(PID, debug_dis);
        mode_flag = 2;
    }
    else
    {
        yaw_TurnTo(robot_heading, 0);
    }
        */
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
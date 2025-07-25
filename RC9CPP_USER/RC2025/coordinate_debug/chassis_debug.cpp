#include "chassis_debug.h"

chassis_adjust_xbox::chassis_adjust_xbox(imu *imu_ptr_, imu *ros_imu_, CameraOperation *camera_ops_)
{
    imu_ptr = imu_ptr_;
    ros_imu = ros_imu_;
    camera_ops = camera_ops_;

    center_point.x = 3.808f;
    center_point.y = -14.045f;
}

void chassis_adjust_xbox::calc_error()
{
    static Vector2D now_point;
    static Vector2D dis;

    now_point.x = get_world_x();
    now_point.y = get_world_y();

    dis = center_point - now_point;
    dis_2_center = dis.magnitude();

    nor_dir = dis.normalize();
    center_heading = -atan2f(nor_dir.x, nor_dir.y) * 57.296f;

//    center_heading += 180.0f;
//    if (center_heading > 180.0f)
//    {
//        center_heading -= 360.0f;
//    }
//    center_heading -= offest;
}

bool chassis_adjust_xbox::calc_robot_point()
{
    static Vector2D now_point;
    static Vector2D dis;

    if (robot.x == 0 && robot.y == 0)
    {
        return false;
    }

    now_point.x = get_world_x();
    now_point.y = get_world_y();

    dis = robot_point - now_point;
    dis_2_robot = dis.magnitude() - pass_correct_distance;

    nor_dir_robot = dis.normalize();
    robot_heading = -atan2f(nor_dir_robot.x, nor_dir_robot.y) * 57.296f;
// robot_heading += 180.0f;
// if (robot_heading > 180.0f)
// {
//     robot_heading -= 360.0f;
// }
// robot_heading -= offest;
    return true;
}

void chassis_adjust_xbox::btnY_callback()
{
    if (calc_robot_point())
    {

        Vector2D target(0.0f, 0.0f);
        set_RobotVel(target, 0);

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
    }
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
    Vector2D tvel_((-max_target_robot_vel.x * xbox_msgs.joyLHori_map), (-max_target_robot_vel.y * xbox_msgs.joyLVert_map));
    set_WorldVel(tvel_, 2.5f); // 以后再改
    // set_RobotVel_ACCLE(tvel_, 2.5f); 

    if (btn_select_flag)
    {
        set_RobotW(-(2.0f * xbox_msgs.joyRHori_map), 0);
    }
    else
    {
        yaw_TurnTo(center_heading, 0);
    }

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

    /*     if (abs(center_heading - get_yaw()) < limit_yaw_error && abs(get_chassis_yaw_speed()) < limit_yaw_speed)
        {
            set_RobotW(0.0f, 0);
        }
        else
        {
            yaw_TurnTo(center_heading, 0);
        } */

    if (camera_ops->camera_ready == true)
    {
        set_RobotW(0.0f, 0);
    }
    else
    {
        camera_ops->camera_on();
    }

    // yaw_TurnTo(0, 0);
    if (DirRight_flag)
    {
        // shoot_dis = camera_ops->camera_Y / 1000.0f;
        //  auto_shooter_ptr->camera_auto_byFitter(PID, shoot_dis);

        auto_shooter->set_auto_byFitter(PID, dis_2_center);
        // auto_shooter->set_auto_byDis(PID, debug_dis);

        osDelay(666);
        cnt_flag = 0;
        // mode_flag = 2;
        camera_ops->camera_off();
    }
}

void chassis_adjust_xbox::mode_3()
{
    auto_yunball_ptr->control_turn_motor(xbox_msgs.joyRHori_map);
    auto_yunball_ptr->control_lift_motor(-xbox_msgs.joyRVert_map * 80);
    auto_shooter->set_hand(xbox_msgs.joyLVert_map * 800);

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
    Vector2D tvel_((max_target_robot_vel.x * xbox_msgs.joyLHori_map), (max_target_robot_vel.y * xbox_msgs.joyLVert_map));
    set_WorldVel(tvel_, 2.5f); // 以后再改
}

void chassis_adjust_xbox::xbox_on()
{
    imu_ptr->imu_rst();
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

void chassis_adjust_xbox::xbox_share()
{
    center_point.x = get_world_x();
    center_point.y = get_world_y();
}
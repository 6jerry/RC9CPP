#include "auto_lock.h"

void auto_lock_test::calc_error()
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

void auto_lock_test::mode_2()
{
    Vector2D tvel_((3.0f * xbox_msgs.joyLHori_map), (3.0f * xbox_msgs.joyLVert_map));

    set_RobotVel(tvel_, 0);
    set_RobotW(-(2.0f * xbox_msgs.joyRHori_map), 0);
    planner.reset();
    rst_state();
}

void auto_lock_test::mode_1()
{
    // calc_error();

    Vector2D tvel_(0.0f, 0.15f);

    set_WorldVel(tvel_, 0);

    yaw_TurnTo(0.0f, 0);
}

void auto_lock_test::mode_3()
{
    Vector2D tvel((1.0f * xbox_msgs.joyLHori_map), (1.0f * xbox_msgs.joyLVert_map));
    Fine_tune(xbox_msgs.btnDirUp, xbox_msgs.btnDirDown, xbox_msgs.btnDirLeft, xbox_msgs.btnDirRight);
    calc_error();
    if (planner.isFinished())
    {
        planner.start_plan(2.0f,2.0f,3.0f,tvel.magnitude()*3,0.0f,dis_2_center-radius[cnt_flag],0.0f);
    }
    nor_control.setpoint = radius[cnt_flag];

    //nor_speed = -nor_control.PID_Compute(dis_2_center);
    nor_speed = -planner.plan(dis_2_center-radius[cnt_flag]);

    Vector2D tvel_ = tan_dir * (3.0f * xbox_msgs.joyLHori_map);

    Vector2D nor_vel_ = nor_dir * nor_speed;

    set_WorldVel(tvel_ + nor_vel_, 0);

    yaw_TurnTo(center_heading, 0);
}

void auto_lock_test::xbox_on()
{
    center_point.x = 5.541f;
    center_point.y = 0.85f;
    nor_control.ConfigAll(1.0f, 0.0f, 0.02f, 0.0f, 1.0f, 0.005f, 0.0f);
    imu_ptr->imu_relocate(0.20035f, -0.06079f, 0.0f);
    init_locate();
}
auto_lock_test::auto_lock_test(imu *imu_ptr_)
{
    imu_ptr = imu_ptr_;
}

void auto_lock_test::Fine_tune(bool up, bool down, bool left, bool right)
{
    if(HAL_GetTick() - last_tick > 100)
    {
        if (up)    center_point.y += 0.01f;
        if (down)  center_point.y -= 0.01f;
        if (left)  center_point.x -= 0.01f;
        if (right) center_point.x += 0.01f;
        last_tick = HAL_GetTick();
    }
}
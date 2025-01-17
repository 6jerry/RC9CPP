#include "shoot_xbox.h"

void shoot_xbox::sbtnconfig_init()
{
    btnBConfig = {
        &xbox_msgs.btnB,
        &xbox_msgs.btnB_last,
        &speed_level,
        2,
        ButtonActionType::Increment,
        nullptr};

    btnXConfig = {
        &xbox_msgs.btnX,
        &xbox_msgs.btnX_last,
        &speed_level,
        2,
        ButtonActionType::Decrement,
        nullptr};
    // 按键B使得速度档位减一，按键X使得速度档位加一
}

void shoot_xbox::btn_scan()
{
    handleButton(btnXConfig);
    handleButton(btnBConfig);
    // 实时检查这两个按键的状态
}

void shoot_xbox::process_data()
{
    btn_scan();
    joymap_compute(); // 计算各个线性遥感的映射值

    if (speed_level == 1)
    {
        MAX_ROBOT_SPEED_X = 3.20f;
        MAX_ROBOT_SPEED_Y = 3.20f;
        MAX_ROBOT_SPEED_W = 6.20f;
        MAX_RPM = 200.0f;
        pitcher->set_rpm(MAX_RPM * (xbox_msgs.trigLT_map - xbox_msgs.trigRT_map));
        // shooter->set_rpm(MAX_RPM * (xbox_msgs.trigLT_map - xbox_msgs.trigRT_map));
    }

    if (speed_level == 0)
    {
        MAX_ROBOT_SPEED_X = 3.40f;
        MAX_ROBOT_SPEED_Y = 3.40f;
        MAX_ROBOT_SPEED_W = 6.10f;
        MAX_RPM = 200.0f;
        // lifter->set_rpm(MAX_RPM * (xbox_msgs.trigLT_map - xbox_msgs.trigRT_map))
        // shooter->set_rpm(MAX_RPM * (xbox_msgs.trigLT_map - xbox_msgs.trigRT_map));
    }
    if (speed_level == 2)
    {
        MAX_ROBOT_SPEED_X = 3.96f;
        MAX_ROBOT_SPEED_Y = 3.96f;
        MAX_ROBOT_SPEED_W = 6.08f;
        MAX_RPM = 200.0f;
        shooter->set_rpm(MAX_RPM * (xbox_msgs.trigLT_map - xbox_msgs.trigRT_map));
    }

    control_chassis->switch_chassis_mode(remote_robotv);

    arm_sqrt_f32(xbox_msgs.joyLHori_map * xbox_msgs.joyLHori_map + xbox_msgs.joyLVert_map * xbox_msgs.joyLVert_map, &mapsum);

    if (mapsum > 0.15f)
    {
        control_chassis->if_adjust_heading = true;
    }
    else
    {
        control_chassis->if_adjust_heading = false;
    }
    control_chassis->setrobotv(MAX_ROBOT_SPEED_X * xbox_msgs.joyLHori_map, -MAX_ROBOT_SPEED_Y * xbox_msgs.joyLVert_map, MAX_ROBOT_SPEED_W * xbox_msgs.joyRHori_map);
}

shoot_xbox::shoot_xbox(power_motor *shooter_, power_motor *pitch, power_motor *lifter_, chassis *control_chassis_) : shooter(shooter_), pitcher(pitch), control_chassis(control_chassis_), lifter(lifter_)
{
    sbtnconfig_init();
}
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
        MAX_RPM = 186.0f;
    }

    if (speed_level == 0)
    {
        MAX_RPM = 86.0f;
    }
    if (speed_level == 2)
    {
        MAX_RPM = 286.0f;
    }

    shooter->set_rpm((xbox_msgs.trigLT_map - xbox_msgs.trigRT_map) * MAX_RPM);
}

shoot_xbox::shoot_xbox(power_motor *shooter_, power_motor *pitch) : shooter(shooter_), pitcher(pitch)
{
    sbtnconfig_init();
}
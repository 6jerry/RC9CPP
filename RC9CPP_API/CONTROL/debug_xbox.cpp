#include "debug_xbox.h"



void xbox_debug_base::btnconfig_init()
{
    btnBConfig = {
        &xbox_msgs.btnB,
        &xbox_msgs.btnB_last,
        &mode_flag,
        4,
        ButtonActionType::Increment,
        nullptr};

    btnXConfig = {
        &xbox_msgs.btnX,
        &xbox_msgs.btnX_last,
        &mode_flag,
        4,
        ButtonActionType::Decrement,
        nullptr};

    btnAConfig = {
        &xbox_msgs.btnA,
        &xbox_msgs.btnA_last,
        &start_flag,
        1,
        ButtonActionType::Toggle,
        nullptr};

    btnLBConfig = {
        &xbox_msgs.btnLB,
        &xbox_msgs.btnLB_last,
        &lb_flag,
        1,
        ButtonActionType::Toggle,
        nullptr};

    btnRBConfig = {
        &xbox_msgs.btnRB,
        &xbox_msgs.btnRB_last,
        &rb_flag,
        1,
        ButtonActionType::Toggle,
        nullptr};

    btnXboxConfig = {
        &xbox_msgs.btnXbox,
        &xbox_msgs.btnXbox_last,
        nullptr,
        0,
        ButtonActionType::Custom,
        &xbox::btnXBOX_callback};

    btnYConfig = {
        &xbox_msgs.btnY,
        &xbox_msgs.btnY_last,
        &cnt_flag,
        3,
        ButtonActionType::Toggle,
        nullptr};

    btnDirUpConfig = {
        &xbox_msgs.btnDirUp,
        &xbox_msgs.btnDirUp_last,
        &shoot_title,
        1,
        ButtonActionType::Toggle,
        nullptr};
    btnDirDownConfig = {
        &xbox_msgs.btnDirDown,
        &xbox_msgs.btnDirDown_last,
        &yunball_title,
        1,
        ButtonActionType::Toggle,
        nullptr};

    btnDirLeftConfig = {
        &xbox_msgs.btnDirLeft,
        &xbox_msgs.btnDirLeft_last,
        &just_yun_title,
        1,
        ButtonActionType::Toggle,
        nullptr};
}

void xbox_debug_base::btn_scan()
{
    handleButton(btnAConfig);
    handleButton(btnXConfig);
    handleButton(btnBConfig);
    handleButton(btnLBConfig);
    handleButton(btnRBConfig);
    handleButton(btnDirUpConfig);
    handleButton(btnDirDownConfig);
    handleButton(btnDirLeftConfig);
    handleButton(btnXboxConfig);
    handleButton(btnYConfig);
}

void xbox_debug_base::btnXBOX_callback()
{
    xbox_on();
}

xbox_debug_base::xbox_debug_base()
{
    btnconfig_init();
}

void xbox_debug_base::process_data()
{
    btn_scan();
    joymap_compute();

    if (start_flag == 1)
    {
        switch (mode_flag)
        {
        case 0:
            mode_0();
            break;
        case 1:
            mode_1();
            break;
        case 2:
            mode_2();
            break;
        case 3:
            mode_3();
            break;
        case 4:
            mode_4();
            break;
        default:
            break;
        }
    }
    else if (start_flag == 0)
    {
        not_start();
    }

    if (lb_flag == 1)
    {
        lb_on();
    }
    else if (lb_flag == 0)
    {
        lb_off();
    }

    if (rb_flag == 1)
    {
        rb_on();
    }
    else if (rb_flag == 0)
    {
        rb_off();
    }
}


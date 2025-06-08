#include "yunball_test_xbox.h"

void yunball_test_xbox::process_data()
{
    btn_scan();
    joymap_compute();
    if(btn_start_flag == 1)
    {
        HAL_NVIC_SystemReset();
    }

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
        default:
            break;
        }
    }
    else if (start_flag == 0) not_start();
}

yunball_test_xbox::yunball_test_xbox(imu *imu_ptr_)
{
    btnconfig_init();
    imu_ptr = imu_ptr_;
    max_target_robot_vel.x = 5.0f;
    max_target_robot_vel.y = 5.0f;
	mode_flag = 1;
}

void yunball_test_xbox::btnconfig_init()
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
        &up_flag,
        1,
        ButtonActionType::Toggle,
        nullptr};
    btnDirDownConfig = {
        &xbox_msgs.btnDirDown,
        &xbox_msgs.btnDirDown_last,
        &down_flag,
        1,
        ButtonActionType::Toggle,
        nullptr};

    btnDirLeftConfig = {
        &xbox_msgs.btnDirLeft,
        &xbox_msgs.btnDirLeft_last,
        &left_flag,
        5,
        ButtonActionType::Toggle,
        nullptr};

    btnDirRightConfig = {
        &xbox_msgs.btnDirRight,
        &xbox_msgs.btnDirRight_last,
        &right_flag,
        1,
        ButtonActionType::Toggle,
        nullptr};
		
		btnStartConfig = {
        &xbox_msgs.btnStart,
        &xbox_msgs.btnStart_last,
        &btn_start_flag,
        1,
        ButtonActionType::Toggle,
        nullptr};
}

void yunball_test_xbox::btn_scan()
{
    handleButton(btnAConfig);
    handleButton(btnXConfig);
    handleButton(btnBConfig);
    handleButton(btnLBConfig);
    handleButton(btnRBConfig);
    handleButton(btnXboxConfig);
    handleButton(btnYConfig);
    handleButton(btnDirUpConfig);
    handleButton(btnDirDownConfig);
    handleButton(btnDirLeftConfig);
    handleButton(btnDirRightConfig);
    handleButton(btnStartConfig);
}

void yunball_test_xbox::xbox_on()
{
    //post->imu_rst();
    imu_ptr->imu_relocate(0.0f, 0.0f, 0.0f);
}

void yunball_test_xbox::not_start()  //初始状态
{
    /*turn_motor->set_rpm(0.0f);
    set_claw(false);
    set_lift(false);
    set_push(false);*/
}

void yunball_test_xbox::mode_1()
{
    // 底盘速度控制
    Vector2D tvel_((max_target_robot_vel.x * xbox_msgs.joyLHori_map), (max_target_robot_vel.y * xbox_msgs.joyLVert_map));
    set_RobotVel(tvel_, 4.0f);

    if(xbox_msgs.joyRHori_map == 0.0f)
    {
        Correct_yaw(lock_yaw);
    }
    else
    {
        set_RobotW(-(2.0f * xbox_msgs.joyRHori_map), 0);
        lock_yaw = imu_ptr->get_yaw_rad() * 57.296f;
    }


    //上层
    if(down_flag){auto_yunball_ptr->control_lift(true);}
    else{auto_yunball_ptr->control_lift(false);}

    if(up_flag){auto_yunball_ptr->control_claw(true);}
    else{auto_yunball_ptr->control_claw(false);}

    if(left_flag){
        auto_yunball_ptr->start_yunball();
        left_flag = 0; up_flag = 0; down_flag = 0; right_flag = 0;
    }
    if(right_flag){
        auto_yunball_ptr->start_putball();
        left_flag = 0; up_flag = 0; down_flag = 0; right_flag = 0;
    }
    point_cnt = 0;
}

void yunball_test_xbox::mode_2()
{
   auto_yunball_ptr->control_motor(xbox_msgs.joyRHori_map);
}

/*void yunball_test_xbox::mode_0()
{
    // 底盘速度控制
    Vector2D tvel_(0, (max_target_robot_vel.y * 0.1f));
    set_RobotVel(tvel_, 4.0f);

    if(xbox_msgs.joyRHori_map == 0.0f)
    {
        Correct_yaw(lock_yaw);
    }
    else
    {
        set_RobotW(-(2.0f * xbox_msgs.joyRHori_map), 0);
        lock_yaw = imu_ptr->get_yaw_rad() * 57.296f;
    }

    //上层
    if(left_flag){
        auto_yunball_ptr->start_yunball();
        left_flag = 0; up_flag = 0; down_flag = 0; right_flag = 0;
    }
}*/

void yunball_test_xbox::mode_0()
{
    if(down_flag && point_cnt > 0){point_cnt--; down_flag = 0;}
    if(up_flag && point_cnt < 6){point_cnt++; up_flag = 0;}

    pp_track_point(track_point[point_cnt]);

    if(left_flag){
        auto_yunball_ptr->start_yunball();
        left_flag = 0; up_flag = 0; down_flag = 0; right_flag = 0;
    }
}

void yunball_test_xbox::add_autoyunball(auto_yunball *auto_yunball_)
{
    auto_yunball_ptr = auto_yunball_;
}

#include "yunball_test_xbox.h"

void yunball_test_xbox::process_data()
{
    btn_scan();
    joymap_compute();

    if (start_flag == 1)
    {
        switch (mode_flag)
        {
        case 0:
            mode_0(); // 手动控制
            break;
        case 1:
            mode_1(); // 按键控制
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
        not_start();
}

yunball_test_xbox::yunball_test_xbox(imu *imu_ptr_)
{
    btnconfig_init();
    imu_ptr = imu_ptr_;
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
        7,
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
}

void yunball_test_xbox::btnXBOX_callback()
{
    xbox_on();
}

void yunball_test_xbox::xbox_on()
{
    imu_ptr->imu_rst();
}

void yunball_test_xbox::not_start()
{
    Vector2D tvel_(0.0f, 0.0f);
    set_WorldVel(tvel_, 2.5f);
    set_RobotW(0, 0);
}

void yunball_test_xbox::mode_0()
{
    if (lb_flag)
    {
        auto_yunball_ptr->start_yunball();
        lb_flag = 0;
    }
    set_RobotW(-(2.0f * w), 0);
    Vector2D tvel_(Vx, Vy);
    set_WorldVel(tvel_, 0);
}

void yunball_test_xbox::mode_1()
{
    auto_yunball_ptr->control_claw(up_flag);
    auto_yunball_ptr->control_push(down_flag);
    auto_yunball_ptr->control_lift_motor(-xbox_msgs.joyRVert_map * 2.0f);
    auto_yunball_ptr->control_turn_motor(-xbox_msgs.joyRHori_map * 2.0f);
    if (lb_flag)
    {
        auto_yunball_ptr->start_yunball();
        lb_flag = 0;
    }
}

void yunball_test_xbox::mode_2()
{
    Vector2D tvel_((5.0f * xbox_msgs.joyLHori_map), (5.0f * xbox_msgs.joyLVert_map));
    set_WorldVel(tvel_, 0); 
    set_RobotW(-(2.0f * xbox_msgs.joyRHori_map), 0);

    if (lb_flag)
    {
        auto_yunball_ptr->start_yunball();
        lb_flag = 0;
    }
}

void yunball_test_xbox::mode_3()
{
    set_WorldVel(velocities[cnt_flag], 0);
    set_RobotW(w_values[cnt_flag], 0);

    if (cnt_flag == yun_flag)
    {
        auto_yunball_ptr->start_double_yunball();

        yun_flag++;
    }
    if (cnt_flag == 0)
    {
        yun_flag = 1;
    }
}

void yunball_test_xbox::mode_4()
{	
//	 if(left_flag)
//    {	
//        if(cnt_flag == 6)
//         cnt_flag = 0; 
//        else
//         cnt_flag++; 
//         
//        rst_state();
//        left_flag = 0;
//    }

//    if(right_flag)
//    {	
//		
//        if(cnt_flag == 0)
//         cnt_flag = 6; 
//        else
//         cnt_flag--; 
//         
//        rst_state();
//        right_flag = 0;
//    }
//	
//    pp_track_point(t_points[cnt_flag]);
//	Vector2D err=imu_ptr->get_world_pos()-t_points[cnt_flag];
//	

//    if (lb_flag)
//    {
//        auto_yunball_ptr->start_yunball();
//        lb_flag = 0;
//    }
//    /*if (calc_dis(t_points[cnt_flag]) > change_dis)
//    {
//        pp_track_point(t_points[cnt_flag]);
//        set_RobotW(0.0f, 0);
//    }else
//    {
//        set_RobotVel(Vector2D(0.0f, 0.0f), 0);
//        set_RobotW(0.0f, 0);
//        rst_state();
//    }*/
}
void yunball_test_xbox::add_autoyunball(auto_yunball *auto_yunball_)
{
    auto_yunball_ptr = auto_yunball_;
}

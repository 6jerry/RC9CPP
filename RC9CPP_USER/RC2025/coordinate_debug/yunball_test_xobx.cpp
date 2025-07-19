#include "yunball_test_xbox.h"

//static uint8_t tile[4] = {0x00,0x00,0x80,0x7F};

void yunball_test_xbox::process_data()
{
    btn_scan();
    joymap_compute();

    if (start_flag == 1)
    {
        switch (mode_flag)
        {
        case 0:
            mode_0();       //手动控制
            break;
        case 1:
            mode_1();       //按键控制
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
    else if (start_flag == 0) not_start();
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
        8,
        ButtonActionType::Increment,
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
    imu_ptr->imu_relocate(0.0f, 0.0f, 0.0f);
}

void yunball_test_xbox::not_start()  //初始状态
{
    Vector2D tvel_(0.0f, 0.0f);
    set_WorldVel(tvel_, 2.5f); 
    set_RobotW(0, 0);
}

void yunball_test_xbox::mode_0()
{
//    if(down_flag){auto_yunball_ptr->control_lift(true);}
//    else{auto_yunball_ptr->control_lift(false);}

//    if(up_flag){auto_yunball_ptr->control_claw(true);}
//    else{auto_yunball_ptr->control_claw(false);}

//    if(left_flag){
//        auto_yunball_ptr->start_yunball();
//        left_flag = 0; up_flag = 0; down_flag = 0; right_flag = 0;
//    }
//    /*if(right_flag){
//        auto_yunball_ptr->start_putball();
//        left_flag = 0; up_flag = 0; down_flag = 0; right_flag = 0;
//    }*/
//    /*turn_motor->set_rpm(-xbox_msgs.joyRHori_map * max_turn_speed);
//    set_claw(up_flag);
//    set_lift(down_flag);
//    if(left_flag == 1) 
//    {
//        turn_motor->set_rpm(0.0f);
//       yunball(); 
//        left_flag = 0;
//    }*/
	if (lb_flag)
    {
        auto_yunball_ptr->start_yunball();
        lb_flag = 0;
    }
	    set_RobotW(-(2.0f * zhuan_su), 0);
	Vector2D tvel_(Vx, Vy);
    set_WorldVel(tvel_, 0); // 以后再改
    
}

void yunball_test_xbox::mode_1()
{
//   auto_yunball_ptr->control_motor(xbox_msgs.joyRHori_map);
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
    Vector2D tvel_((5.0f * xbox_msgs.joyLHori_map), (5.0f  * xbox_msgs.joyLVert_map));
    set_WorldVel(tvel_, 2.5f); // 以后再改
    set_RobotW(-(2.0f * xbox_msgs.joyRHori_map), 0);

    if (lb_flag)
    {
        auto_yunball_ptr->start_yunball();
        lb_flag = 0;
    }
    if (rb_flag)
    {
        //if (auto_yunball_ptr->start_putball())
        //{
            rb_flag = 0;
        //}
    }
}

void yunball_test_xbox::mode_3()
{	
	if (lb_flag)
    {
        auto_yunball_ptr->start_yunball();
        lb_flag = 0;
    }
	
	if (cnt_flag == 0){
		Vx=-0.22;
    Vy=0.22;
	Vector2D tvel_(Vx, Vy);
    set_WorldVel(tvel_, 0); 
    set_RobotW(0, 0);
		
		if (yun_flag==1){
			yun_flag = 2;
		auto_yunball_ptr->start_yunball();
		}
		
		
	}
	
	
	if (cnt_flag == 1){
		Vx=0;
    Vy=0.55;
	Vector2D tvel_(Vx, Vy);
    set_WorldVel(tvel_, 0); 
    set_RobotW(-(2.0f * 0.10), 0);

		if (yun_flag==2){
		auto_yunball_ptr->start_yunball();
			yun_flag = 3;
		}
	}
	
	if (cnt_flag == 2){
		Vx=0.55;
    Vy=0;
	Vector2D tvel_(Vx, Vy);
    set_WorldVel(tvel_, 0); 
    set_RobotW(-(2.0f * 0.10), 0);

		if (yun_flag==3){
			
		auto_yunball_ptr->start_yunball();
			yun_flag = 4;
		}
	}
	
	if (cnt_flag == 3){
		Vx=0.55;
    Vy=0;
	Vector2D tvel_(Vx, Vy);
    set_WorldVel(tvel_, 0);
    set_RobotW(-(2.0f * 0.10), 0);
		if (yun_flag==4){
		auto_yunball_ptr->start_yunball();
			yun_flag = 5;
		}
	}
	
	if (cnt_flag == 4){
		Vx=0;
    Vy=-0.55;
	Vector2D tvel_(Vx, Vy);
    set_WorldVel(tvel_, 0); 
    set_RobotW(-(2.0f * 0.10), 0);
		if (yun_flag==5){
		auto_yunball_ptr->start_yunball();
			yun_flag = 6;
		}
	}
	
	if (cnt_flag == 5){
		Vx=-0.22;
    Vy=-0.22;
	Vector2D tvel_(Vx, Vy);
    set_WorldVel(tvel_, 0); 
    set_RobotW(-(2.0f * 0.10), 0);
		if (yun_flag==6){
		auto_yunball_ptr->start_yunball();
			yun_flag = 7;
		}
	}
	
	if (cnt_flag == 6){
		Vx=-0.63;
    Vy=0;
	Vector2D tvel_(Vx, Vy);
    set_WorldVel(tvel_, 0); 
    set_RobotW(0, 0);
		if (yun_flag==7){
		auto_yunball_ptr->start_yunball();
			yun_flag = 8;
		}
	}
	
	if (cnt_flag == 7){
		Vx=0;
    Vy=0;
	Vector2D tvel_(Vx, Vy);
    set_WorldVel(tvel_, 0); 
    set_RobotW(0, 0);
		if (yun_flag==8){
		
			yun_flag = 1;
		}
	}
	
	if (cnt_flag == 8){
		cnt_flag=0;
		
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




void yunball_test_xbox::add_io(GPIO_TypeDef *turn_port_, uint16_t turn_pin_)
{
    turn_port = turn_port_;
    turn_pin = turn_pin_;
}

void yunball_test_xbox::add_motor(m3508p *turn_motor_)
{
    turn_motor = turn_motor_;
}

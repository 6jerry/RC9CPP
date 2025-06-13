#include "yunball_test_xbox.h"

//static uint8_t tile[4] = {0x00,0x00,0x80,0x7F};

void yunball_test_xbox::process_data()
{
    btn_scan();
    joymap_compute();

    /*float_data.value = turn_motor->rcurrent;
    HAL_UART_Transmit(&huart5, float_data.bytes, (uint16_t)sizeof(float), 100);
    HAL_UART_Transmit(&huart5, tile, sizeof(tile), 100);*/
    /*if(HAL_GPIO_ReadPin(turn_port, turn_pin))
    {
        HAL_UART_Transmit(&huart5, (uint8_t*)"ok", 2, 100);
    }
    else
    {
        HAL_UART_Transmit(&huart5, (uint8_t*)"no", 2, 100);
    }*/

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
        default:
            break;
        }
    }
    else if (start_flag == 0) not_start();
}

yunball_test_xbox::yunball_test_xbox()
{
    btnconfig_init();
	mode_flag = 0;
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



void yunball_test_xbox::not_start()  //初始状态
{
    /*turn_motor->set_rpm(0.0f);
    set_claw(false);
    set_lift(false);
    set_push(false);*/
}

void yunball_test_xbox::mode_0()
{
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
    /*turn_motor->set_rpm(-xbox_msgs.joyRHori_map * max_turn_speed);
    set_claw(up_flag);
    set_lift(down_flag);
    if(left_flag == 1) 
    {
        turn_motor->set_rpm(0.0f);
        yunball(); 
        left_flag = 0;
    }*/
}

void yunball_test_xbox::mode_1()
{
    /*if(right_flag == 1)
    {
        putball();
        right_flag = 0;
    }
    if(left_flag == 1) 
    {
        turn_motor->set_rpm(0.0f);
        yunball(); 
        left_flag = 0;
    }*/
   auto_yunball_ptr->control_motor(xbox_msgs.joyRHori_map);
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
/*
void yunball_test_xbox::yunball()
{
    set_claw(true);
    set_push(true);
    osDelay(200); 
    set_push(false);
    osDelay(400);
    set_claw(false);
}
void yunball_test_xbox::putball()
{
    set_lift(true);
    turn_motor->set_pos_speedplan(-180.0f, 20.0f, 10.0f, 10.0f, 0.0f);
    osDelay(1700);
    set_claw(true);
    osDelay(200);
    turn_motor->set_pos_speedplan(-30.0f, 20.0f, 10.0f, 10.0f, 0.0f);
    osDelay(1500);
    set_lift(false);
}

void yunball_test_xbox::set_claw(bool if_open) {HAL_GPIO_WritePin(claw_port, claw_pin, if_open ? GPIO_PIN_RESET : GPIO_PIN_SET);}
void yunball_test_xbox::set_lift(bool if_up) {HAL_GPIO_WritePin(lift_port, lift_pin, if_up ? GPIO_PIN_SET : GPIO_PIN_RESET);}
void yunball_test_xbox::set_push(bool if_push) {HAL_GPIO_WritePin(push_port, push_pin, if_push ? GPIO_PIN_SET : GPIO_PIN_RESET);}*/
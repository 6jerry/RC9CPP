#include "LJH_xbox_test.h"

xbox_controller::xbox_controller()
{
    btnAConfig = {
        &xbox_msgs.btnA,
        &xbox_msgs.btnA_last,
        &test1,
        1,
        ButtonActionType::Toggle,
        nullptr};

    btnYConfig = {
        &xbox_msgs.btnY,
        &xbox_msgs.btnY_last,
        &test2,
        1,
        ButtonActionType::Toggle,
        nullptr};

    btnRBConfig = {
        &xbox_msgs.btnRB,
        &xbox_msgs.btnRB_last,
        &test3,
        1,
        ButtonActionType::Toggle,
        nullptr};

    btnLBConfig = {
        &xbox_msgs.btnLB,
        &xbox_msgs.btnLB_last,
        &test4,
        1,
        ButtonActionType::Toggle,
        nullptr};
}

void xbox_controller::process_data()
{
    btn_scan();   // 检测按钮状态
    joymap_compute();     // 检测摇杆状态

    if(test1 == 1)
    {
        //事件1 - on
        motor1->set_rpm(xbox_msgs.joyRVert_map * max_lifter_speed);
        motor2->set_rpm(xbox_msgs.joyLHori_map * max_turn_speed);
        motor3->set_rpm(0.0f);
        motor4->set_rpm(0.0f);
    }
    else if(test1 == 0)
    {
        //事件1 - off
        motor1->set_rpm(0.0f);
        motor2->set_rpm(0.0f);
        motor3->set_rpm(xbox_msgs.joyLVert_map * max_pithcer_speed);
        motor4->set_rpm((xbox_msgs.trigLT_map - xbox_msgs.trigRT_map) * max_shooter_speed);
    }

    if(test2 == 1)
    {
        //事件2 - on
        motor1->set_dis_speedplan(700,);
    }
    else if(test2 == 0)
    {
        //事件2 - off
    }

    if(test3 == 1)
    {
        //事件3 - on
        HAL_GPIO_WritePin(port_1, pin_1, GPIO_PIN_SET);
    }
    else if(test3 == 0)
    {
        //事件3 - off
        HAL_GPIO_WritePin(port_1, pin_1, GPIO_PIN_RESET);
    }

    if(test4 == 1)
    {
        //事件4 - on
        HAL_GPIO_WritePin(port_2, pin_2, GPIO_PIN_SET);
    }
    else if(test4 == 0)
    {
        //事件4 - off
        HAL_GPIO_WritePin(port_2, pin_2, GPIO_PIN_RESET);
    }
}

void xbox_controller::btn_scan()
{
    handleButton(btnAConfig);
    handleButton(btnYConfig);
    handleButton(btnRBConfig);
    handleButton(btnLBConfig);      
}

void xbox_controller::add_motor(power_motor* motor1_, power_motor* motor2_, power_motor* motor3_, power_motor* motor4_)
{
    motor1 = motor1_;
    motor2 = motor2_;
    motor3 = motor3_;
    motor4 = motor4_;
}

void xbox_controller::add_trigger(GPIO_TypeDef *port_1, uint16_t pin_1, GPIO_TypeDef *port_2, uint16_t pin_2)
{
    this->port_1 = port_1;
    this->pin_1 = pin_1;
    this->port_2 = port_2;
    this->pin_2 = pin_2;
}
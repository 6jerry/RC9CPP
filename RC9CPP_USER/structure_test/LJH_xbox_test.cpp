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
    }
    else if(test1 == 0)
    {
        //事件1 - off
    }

    if(test2 == 1)
    {
        //事件2 - on
    }
    else if(test2 == 0)
    {
        //事件2 - off
    }

    if(test3 == 1)
    {
        //事件3 - on
    }
    else if(test3 == 0)
    {
        //事件3 - off
    }

    if(test4 == 1)
    {
        //事件4 - on
    }
    else if(test4 == 0)
    {
        //事件4 - off
    }
}

void xbox_controller::btn_scan()
{
    handleButton(btnAConfig);
    handleButton(btnYConfig);
    handleButton(btnRBConfig);
    handleButton(btnLBConfig);      
}

void xbox_controller::add_motor(power_motor* motor1, power_motor* motor2, power_motor* motor3, power_motor* motor4)
{
    this->motor1 = motor1;
    this->motor2 = motor2;
    this->motor3 = motor3;
    this->motor4 = motor4;
}

void xbox_controller::add_trigger(GPIO_TypeDef *port_1, uint16_t pin_1, GPIO_TypeDef *port_2, uint16_t pin_2)
{
    this->port_1 = port_1;
    this->pin_1 = pin_1;
    this->port_2 = port_2;
    this->pin_2 = pin_2;
}
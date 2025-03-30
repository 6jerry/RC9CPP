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


    if(test2 == 1)
    {
        //HAL_GPIO_WritePin(port_3, pin_3, GPIO_PIN_SET);
    }
    else if(test2 == 0)
    {
        //HAL_GPIO_WritePin(port_3, pin_3, GPIO_PIN_RESET);
    }

    if(test4 == 1)          //自动运球
    {
        auto_yunball();
        motor3->set_rpm(0.0f);
        control_shooter_speed(0.0f);
    }
    else if(test4 == 0)     //手动控制
    {
        step = catch_point;         /*完成一轮自动运球后将标志位设置成就绪态*/

        if(test3 == 1)      //校准
        {
            if (HAL_GPIO_ReadPin(port_1, pin_1) != 0)
            {
                motor1->set_rpm(-60.0f);
            }
            else if (HAL_GPIO_ReadPin(port_1, pin_1) == 0)
            {
                motor1->set_rpm(0.0f);
                motor1->relocate_dis(0.0f);
                test3 = 0;
            }
        }

        else if(test3 == 0)      //手操
        {
            if(test1 == 1)
            {
                motor1->set_rpm(xbox_msgs.joyRVert_map * max_lifter_speed);
                motor2->set_rpm(xbox_msgs.joyLHori_map * max_turn_speed);
                motor3->set_rpm(0.0f);
                control_shooter_speed(0.0f);
            }
            else if(test1 == 0)
            {
                motor1->set_rpm(0.0f);
                motor2->set_rpm(0.0f);
                motor3->set_rpm(xbox_msgs.joyLVert_map * max_pithcer_speed);
                control_shooter_speed((xbox_msgs.trigLT_map - xbox_msgs.trigRT_map) * max_shooter_speed / 2.0f);
            }
        }
    }
}

void xbox_controller::auto_yunball()  //纯运球
{
    switch(step)
    {
        case reset:
            motor1->set_dis_speedplan(500,max_lifter_speed,400,500,0);
            if (abs(500 - motor1->get_dis()) <= 10.0f)
            {
                motor1->dis_speedplan_restart();
                test4 = 0;
            }
        break;
        
        case catch_point:
            motor1->set_dis_speedplan(850,2000,1500,1500,0);
            if (abs(750 - motor1->get_dis()) <= 50.0f)
            {
                step = throw_point;
                motor1->dis_speedplan_restart();
								HAL_GPIO_WritePin(port_3, pin_3, GPIO_PIN_RESET);
            }
        break;
        
        case throw_point:
            motor1->set_dis_speedplan(300,2000,2000,2000,500);
            if (abs(300 - motor1->get_dis()) <= 20.0f)
            {
				motor1->set_rpm(0.0f);
				time_cnt++;
                if(time_cnt>=1)
                {
                    step = reset;
                    motor1->dis_speedplan_restart();
                    HAL_GPIO_WritePin(port_3, pin_3, GPIO_PIN_SET);
                    time_cnt=0;
                }
            }
        break;
        
        default:
        break;
    }
}

void xbox_controller::auto_yunball_pro()   //运球+放球
{
    switch(step)
    {
        case reset:
            motor1->set_dis_speedplan(500,max_lifter_speed,400,500,0);
            if (abs(500 - motor1->get_dis()) <= 10.0f)
            {
                motor1->dis_speedplan_restart();
                test4 = 0;
            }
        break;
        break;
        
        case catch_point:
            motor1->set_dis_speedplan(750,1500,1000,1000,0);
            if (abs(750 - motor1->get_dis()) <= 10.0f)
            {
                step = throw_point;
                motor1->dis_speedplan_restart();
				HAL_GPIO_WritePin(port_3, pin_3, GPIO_PIN_RESET);
            }
        break;
        
        case throw_point:
            motor1->set_dis_speedplan(300,1500,1000,1000,0);
            if (abs(300 - motor1->get_dis()) <= 20.0f)
            {
				motor1->set_rpm(0.0f);
				time_cnt++;
                if(time_cnt>=50)
                {
                    step = turn_point;
                    motor1->dis_speedplan_restart();
                    HAL_GPIO_WritePin(port_3, pin_3, GPIO_PIN_SET);
                    time_cnt=0;
                }
            }
        break;

        case turn_point:
            motor1->set_dis_speedplan(750,1500,1000,1000,0);
            if (abs(750 - motor1->get_dis()) <= 10.0f)
            {
                step = turn_to;
                motor1->dis_speedplan_restart();
            }
        break;

        case turn_to:
            motor2->set_pos_speedplan(-90.0f,30.0f,10.0f,10.0f,0.0f);
            if(abs(-90.0f - motor2->get_pos()) <= 5.0f)
            {
                motor2->set_rpm(0.0f);
                motor2->pos_speedplan_restart();
                time_cnt++;
                if(time_cnt>=50)
                {                    
                    motor2->dis_speedplan_restart();
                    HAL_GPIO_WritePin(port_3, pin_3, GPIO_PIN_RESET);
                }
                if(time_cnt>=100)
                {
                    step = turn_back;
                    time_cnt = 0;
                }
            }
        break;

        case turn_back:
            motor2->set_pos_speedplan(90.0f,30.0f,10.0f,10.0f,0.0f);
            if(abs(90.0f - motor2->get_pos()) <= 5.0f)
            {
                motor2->set_rpm(0.0f);
                motor2->pos_speedplan_restart();
                step = reset;
                HAL_GPIO_WritePin(port_3, pin_3, GPIO_PIN_SET);
            }
        break;
        
        default:
        break;
    }
}


void xbox_controller::control_shooter_speed(float speed)
{
    static float prev_speed = 0.0f;
    static uint32_t last_tick = 0;
    static bool pending_pin_set = false;

    // 状态变化检测
    if (speed != prev_speed) {
        // 从0变到非0的情况
        if (prev_speed == 0.0f && speed != 0.0f) {
            HAL_GPIO_WritePin(port_2, pin_2, GPIO_PIN_RESET); 
            last_tick = HAL_GetTick();
            pending_pin_set = false;
        }
        // 从非0变到0的情况
        else if (prev_speed != 0.0f && speed == 0.0f) {
            motor4->set_rpm(0.0f);    
            last_tick = HAL_GetTick();
            pending_pin_set = true;  
        }
        prev_speed = speed;
    }

    // 延时处理
    if (HAL_GetTick() - last_tick >= 100) {
        if (pending_pin_set) {
            HAL_GPIO_WritePin(port_2, pin_2, GPIO_PIN_SET);
            pending_pin_set = false;
        }
        else if (speed != 0.0f) {
            motor4->set_rpm(speed);
        }
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

void xbox_controller::add_trigger(GPIO_TypeDef *port_1, uint16_t pin_1, GPIO_TypeDef *port_2, uint16_t pin_2,GPIO_TypeDef *port_3, uint16_t pin_3,GPIO_TypeDef *port_4, uint16_t pin_4)
{
    this->port_1 = port_1;
    this->pin_1 = pin_1;
    this->port_2 = port_2;
    this->pin_2 = pin_2;
    this->port_3 = port_3;
    this->pin_3 = pin_3;
    this->port_4 = port_4;
    this->pin_4 = pin_4;
}
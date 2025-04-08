#include "rc_test_xbox.h"

void yun_ball_xbox::btnconfig_init()
{

    btnAConfig = {
        &xbox_msgs.btnA,
        &xbox_msgs.btnA_last,
        &if_motor_start,
        1,
        ButtonActionType::Toggle,
        nullptr};

    btnRBConfig = {
        &xbox_msgs.btnRB,
        &xbox_msgs.btnRB_last,
        &trigger_start,
        1,
        ButtonActionType::Toggle,
        nullptr};

    btnYConfig = {
        &xbox_msgs.btnY,
        &xbox_msgs.btnY_last,
        &auto_shooter,
        1,
        ButtonActionType::Toggle,
        nullptr};

    btnLBConfig = {
        &xbox_msgs.btnLB,
        &xbox_msgs.btnLB_last,
        &shooter_trigger,
        1,
        ButtonActionType::Toggle,
        nullptr};

    btnDirUpConfig = {
        &xbox_msgs.btnDirUp,
        &xbox_msgs.btnDirUp_last,
        &yun_trigger,
        1,
        ButtonActionType::Toggle,
        nullptr};
    btnDirDownConfig = {
        &xbox_msgs.btnDirDown,
        &xbox_msgs.btnDirDown_last,
        &pithcer_status,
        1,
        ButtonActionType::Increment,
        nullptr};

    btnDirLeftConfig = {
        &xbox_msgs.btnDirLeft,
        &xbox_msgs.btnDirLeft_last,
        &lifter_status,
        4,
        ButtonActionType::Increment,
        nullptr};
}

void yun_ball_xbox::btn_scan()
{
    handleButton(btnAConfig);
    handleButton(btnRBConfig);
    handleButton(btnYConfig);
    handleButton(btnLBConfig);
    handleButton(btnDirUpConfig);
    handleButton(btnDirDownConfig);
    handleButton(btnDirLeftConfig);
}

yun_ball_xbox::yun_ball_xbox()
{
    btnconfig_init();
}

void yun_ball_xbox::process_data()
{
    btn_scan();
    joymap_compute();

    // 获取拉伸距离和俯仰角度
    shoot_disdance = encoder->get_absolute_distance();
    wit_imu->Get_Data();
    shoot_pitch_angle = wit_imu->Pitch_angle;
    // 标准俯仰-1.141

    if (yun_trigger == 1)
    {
        HAL_GPIO_WritePin(yun_port, yun_pin, GPIO_PIN_SET);
    }
    else if (yun_trigger == 0)
    {
        HAL_GPIO_WritePin(yun_port, yun_pin, GPIO_PIN_RESET);
    }

    if (trigger_start == 1)
    {
        HAL_GPIO_WritePin(trigger_port, trigger_pin, GPIO_PIN_SET);
    }
    else if (trigger_start == 0)
    {
        HAL_GPIO_WritePin(trigger_port, trigger_pin, GPIO_PIN_RESET);
    }

    if (shooter_trigger == 1)
    {
        HAL_GPIO_WritePin(shooter_port, shooter_pin, GPIO_PIN_SET);
    }
    else if (shooter_trigger == 0)
    {
        HAL_GPIO_WritePin(shooter_port, shooter_pin, GPIO_PIN_RESET);
    }

    if (if_motor_start == 1)
    {
        // 手动模式
        if (auto_shooter == 0)
        {

            lfter_motor->set_rpm(xbox_msgs.joyRVert_map * max_lifter_speed);
            turn_motor->set_rpm(-xbox_msgs.joyRHori_map * max_turn_speed);

            shooter_motor->set_rpm(xbox_msgs.joyLVert_map * max_shooter_speed);

            pithcer_motor->set_rpm(-(xbox_msgs.trigLT_map - xbox_msgs.trigRT_map) * max_pithcer_speed);

            // float send_data[1] = {encoder->get_absolute_distance()};
            // test_port->sendFloatData(1, send_data, 1);
        }
        // 自动模式
        else if (auto_shooter == 1)
        {
            // 0.1720
            //  0.1620
            //  0.1920
            // 俯仰状态
            if (pithcer_status == 0)
            {
                pithcer_motor->set_rpm(0.0f);
            }
            else if (pithcer_status == 1)
            {
                adjust_pitcher(-1.141f);
            }

            // 拉伸状态
            if (lifter_status == 0)
            {
                shooter_motor->set_rpm(0.0f);
            }
            else if (lifter_status == 1)
            {

                if (Read_GPIO_State() == GPIO_PIN_RESET)
                {
                    // 触发微动
                    shooter_motor->set_rpm(0.0f);
                    lifter_status == 0;
                }

                adjust_lifter(inital);
            }
            else if (lifter_status == 2)
            {
                adjust_lifter(one);
            }
            else if (lifter_status == 3)
            {
                adjust_lifter(two);
            }
            else if (lifter_status == 4)
            {
                adjust_lifter(three);
            }
        }
    }
    else if (if_motor_start == 0)
    {
        lfter_motor->set_rpm(0.0f);
        turn_motor->set_rpm(0.0f);
        shooter_motor->set_rpm(0.0f);
        pithcer_motor->set_rpm(0.0f);
    }
}
void yun_ball_xbox::add_motor(power_motor *lfter_motor_, power_motor *turn_motor_, power_motor *shooter_motor_, power_motor *pithcer_motor_)
{
    lfter_motor = lfter_motor_;
    turn_motor = turn_motor_;
    shooter_motor = shooter_motor_;
    pithcer_motor = pithcer_motor_;
}

void yun_ball_xbox::add_trigger(GPIO_TypeDef *trigger_port_, uint16_t trigger_pin_, GPIO_TypeDef *shooter_port_,
                                uint16_t shooter_pin_, uint16_t yun_pin_, GPIO_TypeDef *yun_port_, uint16_t stop_pin_, GPIO_TypeDef *stop_port_)
{
    trigger_port = trigger_port_;
    trigger_pin = trigger_pin_;
    shooter_port = shooter_port_;
    shooter_pin = shooter_pin_;
    yun_port = yun_port_;
    yun_pin = yun_pin_;
    stop_pin = stop_pin_;
    stop_port = stop_port_;
}

void yun_ball_xbox::add_laser(imu *laser_)
{
    laser = laser_;
}

void yun_ball_xbox::add_encoder(Encoder *encoder_)
{
    encoder = encoder_;
}
void yun_ball_xbox::add_serial_studio(serial_studio *serial_studio_)
{
    test_port = serial_studio_;
}
void yun_ball_xbox::add_imu(wit_gyro *imu_)
{
    wit_imu = imu_;
}

void yun_ball_xbox::adjust_pitcher(float pitch_angle)
{

    if (shoot_pitch_angle > pitch_angle)
    {
        pithcer_motor->set_rpm(200.0f);
    }
    else
    {
        pithcer_motor->set_rpm(-200.0f);
    }

    if (shoot_pitch_angle > pitch_angle - 0.005f && shoot_pitch_angle < pitch_angle + 0.005f)
    {
        pithcer_status = 0;
        pithcer_motor->set_rpm(0.0f);
    }
}
void yun_ball_xbox::adjust_lifter(float lifter_distance)
{

    // 检查棘轮扳机
    if (trigger_start == 1)
    {
        trigger_start = 0;
        HAL_GPIO_WritePin(trigger_port, trigger_pin, GPIO_PIN_RESET);
    }

    // 检查发射扳机
    if (shooter_trigger == 1)
    {
        shooter_trigger = 0;
        HAL_GPIO_WritePin(shooter_port, shooter_pin, GPIO_PIN_RESET);
    }

    // 使用梯形规划
    if (plan_flag == 0)
    {
        planer.start_plan(max_acc, max_dcc, max_speed, inital_speed, final_speed, shoot_disdance * 10000, lifter_distance * 10000);
        plan_flag = 1;
    }
    shooter_motor->set_rpm(-planer.plan(shoot_disdance * 10000));

    // 到达终点锁住
    if (shoot_disdance > lifter_distance - 0.003f && shoot_disdance < lifter_distance + 0.003f)
    {
        lifter_status = 0;
        shooter_motor->set_rpm(0.0f);
        HAL_GPIO_WritePin(trigger_port, trigger_pin, GPIO_PIN_SET);
        trigger_start = 1;
        plan_flag = 0;
    }
}

// 读取GPIO状态
uint32_t yun_ball_xbox::Read_GPIO_State(void)
{
    return HAL_GPIO_ReadPin(stop_port, stop_pin);
}
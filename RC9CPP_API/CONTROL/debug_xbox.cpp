#include "debug_xbox.h"

void debug_xbox::btnconfig_init()
{
    btnAConfig = {
        &xbox_msgs.btnA,
        &xbox_msgs.btnA_last,
        &if_start,
        1,
        ButtonActionType::Toggle,
        nullptr};
}

void debug_xbox::btn_scan()
{
    handleButton(btnAConfig);
}

void debug_xbox::process_data()
{
    btn_scan();
    joymap_compute();
    map_value = xbox_msgs.trigLT_map - xbox_msgs.trigRT_map;
}

debug_xbox::debug_xbox()
{
    btnconfig_init();
}

void algorithm_debug::addxbox(debug_xbox *debug_)
{
    debug = debug_;
}

float algorithm_debug::get_input_mapvalue()
{
    return debug->map_value;
}

uint8_t algorithm_debug::get_input_start()
{
    return debug->if_start;
}

void algorithm_debug::pid_send_debuginfo(float target, float now_value)
{
    uint8_t id = 1;
    float data[2] = {target, now_value};
    sendFloatData(id, data, 2);
}
void algorithm_debug::filter_send_debuginfo(float orin_data, float filted_data)
{
    uint8_t id = 1;
    float data[2] = {orin_data, filted_data};
    sendFloatData(id, data, 2);
}
void algorithm_debug::speedplan_send_debuginfo(float target_speed, float now_speed)
{
    uint8_t id = 1;
    float data[2] = {target_speed, now_speed};
    sendFloatData(id, data, 2);
}

void algorithm_debug::add_IO(debug_xbox *debug_, RC9Protocol *port_)
{
    addport(port_);
    addxbox(debug_);
}
void algorithm_debug::DataReceivedCallback(const uint8_t *byteData, const float *floatData, uint8_t id, uint16_t byteCount)
{
    for (int i = 0; i < byteCount / 4; i++)
    {
        temp_param[i] = floatData[i];
    }
}

moters_debug_xbox::moters_debug_xbox()
{
    btnconfig_init();
}

void moters_debug_xbox::btnconfig_init()
{

    btnBConfig = {
        &xbox_msgs.btnB,
        &xbox_msgs.btnB_last,
        &debug_mode,
        3,
        ButtonActionType::Increment,
        nullptr};

    btnXConfig = {
        &xbox_msgs.btnX,
        &xbox_msgs.btnX_last,
        &debug_mode,
        3,
        ButtonActionType::Decrement,
        nullptr};

    btnAConfig = {
        &xbox_msgs.btnA,
        &xbox_msgs.btnA_last,
        &start_flag,
        1,
        ButtonActionType::Toggle,
        nullptr};
}

void moters_debug_xbox::btn_scan()
{
    handleButton(btnAConfig);
    handleButton(btnXConfig);
    handleButton(btnBConfig);
}

void moters_debug_xbox::process_data()
{
    btn_scan();
    joymap_compute();

    if (start_flag == 1)
    {
        switch (debug_mode)
        {
        case 0:
            setted_f = (xbox_msgs.trigLT_map - xbox_msgs.trigRT_map) * max_F;
            debug_motor->set_F(setted_f);

            debug_motor->dis_speedplan_restart();

            break;
        case 1:
            debug_motor->set_rpm((xbox_msgs.trigLT_map - xbox_msgs.trigRT_map) * max_rpm);
            debug_motor->dis_speedplan_restart();

            break;
        case 2:
            debug_motor->set_pos((xbox_msgs.trigLT_map - xbox_msgs.trigRT_map) * 180.0f);

            // debug_motor->set_dis_speedplan(target_dis, max_speed, max_acc, max_dec, final_speed);
            break;
        case 3:
            // debug_motor->set_dis(xbox_msgs.trigRT_map * 720.0f + 100.0f);
            debug_motor->dis_speedplan_restart();
            break;
        }
    }
    else
    {
        debug_motor->set_rpm(0.0f);
    }
}

void moters_debug_xbox::add_motor(power_motor *motor_)
{
    debug_motor = motor_;
}

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

    btnYConfig = {
        &xbox_msgs.btnY,
        &xbox_msgs.btnY_last,
        &init_flag,
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

    if (init_flag == 1)
    {
        //chassis_initialize();
        init_flag = 0;
    }
}

void chassis_adjust_xbox::not_start()
{
    set_RobotVel(Vector2D(0.0f, 0.0f), 0);
    set_RobotW(0.0f, 0);
}

void chassis_adjust_xbox::mode_2()
{
    Vector2D tvel_((3.0f * xbox_msgs.joyLHori_map), (3.0f * xbox_msgs.joyLVert_map));

    set_RobotVel(tvel_, 0);
    set_RobotW(-(3.0f * xbox_msgs.joyRHori_map), 0);
}

void chassis_adjust_xbox::mode_1()
{
    Vector2D tvel_((1.0f * xbox_msgs.joyLHori_map), (1.0f * xbox_msgs.joyLVert_map));

    set_RobotVel(tvel_, 0);
    set_RobotW(-(1.0f * xbox_msgs.joyRHori_map), 0);
}

void chassis_adjust_xbox::mode_3()
{
    Vector2D tvel_((5.0f * xbox_msgs.joyLHori_map), (5.0f * xbox_msgs.joyLVert_map));

    set_RobotVel(tvel_, 0);
    set_RobotW(-(5.0f * xbox_msgs.joyRHori_map), 0);
}

void chassis_adjust_xbox::mode_0()
{
    Vector2D tvel_(0.0f, 0.25f);

    set_RobotVel(tvel_, 0);
    set_RobotW(-(5.0f * xbox_msgs.joyRHori_map), 0);
}

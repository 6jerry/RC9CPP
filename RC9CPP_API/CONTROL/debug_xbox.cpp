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

void chassis_debug_xbox::process_data()
{
    btn_scan();
    joymap_compute();
    // currentState = stateMachine.getState();
    currentState = 0;
    Vector2D worldvel_(full_speed * xbox_msgs.joyLHori_map, full_speed * xbox_msgs.joyLVert_map);
    set_RobotVel(worldvel_, priocode);
    set_RobotW(-full_w * xbox_msgs.joyRHori_map, priocode);
    if (btna_flag == 0)
    {
        claw->ready_catch_ball();
    }
    else if (btna_flag == 1)
    {
        claw->move_ball();
    }
    else
    {
        claw->throw_ball();
    }
}

chassis_debug_xbox::chassis_debug_xbox(float full_speed_, float full_w_) : full_speed(full_speed_), full_w(full_w_)
{
    btn_config();
}

void chassis_debug_xbox::btn_config()
{
    btnAConfig = {
        &xbox_msgs.btnA,
        &xbox_msgs.btnA_last,
        &btna_flag,
        2,
        ButtonActionType::Toggle,
        nullptr};

    btnBConfig = {
        &xbox_msgs.btnB,
        &xbox_msgs.btnB_last,
        &btnb_flag,
        1,
        ButtonActionType::Toggle,
        nullptr};

    btnXConfig = {
        &xbox_msgs.btnX,
        &xbox_msgs.btnX_last,
        &btnx_flag,
        1,
        ButtonActionType::Toggle,
        nullptr};

    btnYConfig = {
        &xbox_msgs.btnY,
        &xbox_msgs.btnY_last,
        &btny_flag,
        1,
        ButtonActionType::Toggle,
        nullptr};

    btnShareConfig = {
        &xbox_msgs.btnShare,
        &xbox_msgs.btnShare_last,
        &btnshare_flag,
        1,
        ButtonActionType::Toggle,
        nullptr};
}

void chassis_debug_xbox::btn_scan()
{
    handleButton(btnAConfig);
    handleButton(btnBConfig);
    handleButton(btnXConfig);
    handleButton(btnYConfig);
    handleButton(btnShareConfig);
}
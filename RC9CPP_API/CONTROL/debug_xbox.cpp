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
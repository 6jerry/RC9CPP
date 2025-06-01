#include "m3508_debug.h"

void m3508_m2006_debug::not_start()
{

    debug_motor->set_current(0.0f);
    float send_datas[4] = {debug_motor->get_rpm(),
                           debug_motor->get_target_rpm(),
                           debug_motor->get_pos(),
                           debug_motor->get_target_pos()};

    msg_send->sendFloatData(1, send_datas, 4);
}

void m3508_m2006_debug::mode_2()

{
    debug_motor->set_rpm(max_rpm * xbox_msgs.joyLHori_map);

    float send_datas[4] = {debug_motor->get_rpm(),
                           debug_motor->get_target_rpm(),
                           debug_motor->get_pos(),
                           debug_motor->get_target_pos()};

    msg_send->sendFloatData(1, send_datas, 4);
}

void m3508_m2006_debug::mode_3()
{
    debug_motor->set_pos(90.0f * xbox_msgs.joyLHori_map);
    float send_datas[4] = {debug_motor->get_rpm(),
                           debug_motor->get_target_rpm(),
                           debug_motor->get_pos(),
                           debug_motor->get_target_pos()};

    msg_send->sendFloatData(1, send_datas, 4);
}

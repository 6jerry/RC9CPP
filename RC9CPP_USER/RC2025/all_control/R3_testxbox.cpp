#include "R3_testxbox.h"

void R3_xbox::mode_1()
{
    shoot_motor_1->send_rpm(move_rpm * xbox_msgs.joyLVert_map);
    shoot_motor_2->send_rpm(move_rpm * xbox_msgs.joyLVert_map);
}

void R3_xbox::not_start()
{
    shoot_motor_1->send_rpm(0.0f);
    shoot_motor_2->send_rpm(0.0f);
}

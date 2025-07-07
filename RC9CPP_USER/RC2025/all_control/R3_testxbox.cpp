#include "R3_testxbox.h"

void R3_xbox::mode_1()
{
    MX_FDCAN2_Init();
    CanDevice::InitAllFiltersNoMask();
    target_rpm = move_rpm * xbox_msgs.joyLVert_map;
    shoot_motor_1->set_rpm(target_rpm);
    // shoot_motor_2->set_rpm(target_rpm);

    shoot_motor_2->set_current(shoot_motor_1->get_target_current());
}

void R3_xbox::not_start()
{
    shoot_motor_1->send_rpm(0.0f);
    shoot_motor_2->send_rpm(0.0f);
}

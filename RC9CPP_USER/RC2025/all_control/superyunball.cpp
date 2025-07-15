#include "superyunball.h"

void super_yunball::process_data()
{
    yunball_motor1->send_rpm(remote_control->left_V_map * max_rpm);
    yunball_motor2->send_rpm(remote_control->left_V_map * max_rpm);
}

void super_yunball::add_motors(power_motor *yunball_motor1_, power_motor *yunball_motor2_)
{
    yunball_motor1 = yunball_motor1_;
    yunball_motor2 = yunball_motor2_;
}

void super_yunball::add_remote(CrsfReceiver *remote_control_)
{
    remote_control = remote_control_;
}
#include "shoot_pid_adjust.h"

void shoot_pid_xbox::not_start()
{
    shoot_motor->send_rpm(0.0f);

    float send_datas[3] = {encoder->get_distance(),
                           target_dis,
                           shoot_motor->get_rpm()};

    msg_send->sendFloatData(1, send_datas, 3);
}

void shoot_pid_xbox::mode_2()
{
    shoot_motor->send_rpm(-max_rpm * xbox_msgs.joyLVert_map);
    float send_datas[3] = {encoder->get_distance() * 1000.0f,
                           target_dis * 1000.0f,
                           shoot_motor->get_rpm()};

    msg_send->sendFloatData(1, send_datas, 3);
}

void shoot_pid_xbox::mode_3()
{
    shoot_control.setpoint = target_dis;

    shoot_motor->send_rpm(-shoot_control.PID_Compute(encoder->get_distance()));
    float send_datas[3] = {encoder->get_distance()*1000.0f,
                           target_dis*1000.0f,
                           shoot_motor->get_rpm()};

    msg_send->sendFloatData(1, send_datas, 3);
}

void shoot_pid_xbox::xbox_on()
{
    uint8_t can_send_data[8] = {0x04, 0x01, 0x06, 0x00, 0x00, 0x00, 0x00, 0x00};
    CanDevice::CAN_Send(1, 0, can_send_data, &hfdcan3);
}
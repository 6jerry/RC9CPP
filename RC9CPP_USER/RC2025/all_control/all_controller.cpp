#include "all_controller.h"

AllController::AllController(UART_HandleTypeDef *huart) : CrsfReceiver(huart)
{
}

void AllController::process_data()
{
    calc_data();
    remote_move();
    //sendAttitude(233.0f, 233.3f, 233.0f);

    //sendBattery(1212.0f, 20.0f, 12.0f, 233);
}

void AllController::remote_move()
{
    Vector2D tvel_(right_H_map * max_x_speed, right_V_map * max_y_speed);
    set_WorldVel(tvel_, 0);
    set_RobotW(left_H_map * max_yaw_speed, 0);
}

void AllController::calc_data()
{
    Vector2D now_point;
    now_point.x = get_world_x();
    now_point.y = get_world_y();

    Vector2D dis = {0, 0};

    dis = center_point - now_point;
    dis_2_center = dis.magnitude();
    nor_dir = dis.normalize();
    heading_2_center = -atan2f(nor_dir.x, nor_dir.y) * 57.296f;

    dis = robot_point - now_point;
    dis_2_center = dis.magnitude();
    nor_dir = dis.normalize();
    heading_2_robot = -atan2f(nor_dir.x, nor_dir.y) * 57.296f;
}

void AllController::DataReceivedCallback(const uint8_t *byteData, const float *floatData, uint8_t id, uint16_t byteCount)
{
}
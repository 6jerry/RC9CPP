#ifndef ALL_CONTROLLER_H
#define ALL_CONTROLLER_H

#ifdef __cplusplus
extern "C"
{
#endif
#include "CrsfReceiver.h"
#include "RC9Protocol.h"
#include "imu.h"
#include "TaskManager.h"
#include "robot_chassis.h"
#include "EncodingStateMachine.h"
#ifdef __cplusplus
}
#endif
#ifdef __cplusplus

class AllController : public RC9subscriber, public ITaskProcessor, public chassis_user
{
public:
    float max_x_speed = 4.0f, max_y_speed = 4.0f, max_yaw_speed = 4.0f;
    Vector2D center_point, robot_point, nor_dir; // 篮筐坐标和友军坐标

    float dis_2_center = 0.0f, dis_2_robot = 0.0f, heading_2_center = 0.0f, heading_2_robot = 0.0f;

    CrsfReceiver *crsf_port = nullptr;

public:
    AllController(UART_HandleTypeDef *huart);
    void process_data();
    void DataReceivedCallback(const uint8_t *byteData, const float *floatData, uint8_t id, uint16_t byteCount) override; // 接收友军坐标

public:
    void remote_move(); // 世界坐标系遥控
    void calc_data();   // 计算两个目标距离和两个朝向

    void all_auto_mdoe();
    void all_auto_yunball();
    void all_auto_shootball();
    void all_auto_giveball();
};

#endif
#endif
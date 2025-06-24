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
#include "auto_shooter.h"
#include "auto_yunball.h"
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
    void process_data();
    void DataReceivedCallback(const uint8_t *byteData, const float *floatData, uint8_t id, uint16_t byteCount) override; // 接收友军坐标

    EncodingStateMachine mode_selector, callback_selector;

public:
    void
    remote_move();    // 世界坐标系遥控
    void calc_data(); // 计算两个目标距离和两个朝向

    // 三种全自动模式
    void all_auto_yunball();
    void all_auto_shootball();
    void all_auto_giveball();

    // 三种纯手动模式
    void all_hand_yunball();
    void all_hand_shootball();
    void all_hand_giveball();

    // 准备模式，在开场或者攻防互换的情况下使用
    void prepare_mode();
    void yun_ball_race(); // 挑战赛模式
    void shoot_ball_race();
};

#endif
#endif
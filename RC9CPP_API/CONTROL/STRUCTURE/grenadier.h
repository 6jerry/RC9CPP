#ifndef GRENADIER_H
#define GRENADIER_H
#ifdef __cplusplus
extern "C"
{
#endif
#include "TaskManager.h"
#include "motor.h"

#include "pid.h"
#include "Vector2D.h"

#ifdef __cplusplus
}
#endif

#ifdef __cplusplus

#define G 9.807f // 重力加速度

enum grenadier_mode
{
    grenadier_standby, // 待机
    remote_move_free,  // 机构可自由移动
    auto_shoot,        // 一键发射

    auto_reload,  // 自动复位,一键发射后自动执行
    remote_shoot, // 由操作者控制的发射
    remote_reload // 手动复位

};

class grenadier : public ITaskProcessor
{
private:
    power_motor *motors[2] = {nullptr}; // 双go1
    grenadier_mode mode = grenadier_standby;
    float horizontal_pos = 0.0f; // 机构的水平位置对应着go1的多少角度

    float length = 0.0f, weight = 0.0f; // 投射臂的长度和重量

    void G_torque_compute(); // 计算实时重力矩

public:
    float G_torque = 0.0f, relative_angle = 0.0f; // 投射臂的重力矩和相对角度，位于水平位置上方为正，位于水平位置下方为负
    void process_data();

    bool auto_shoot(float angle, float speed); // 出射角度(相对于水平角度)，出射线速度,m/s
    void remote_reload();                      // 调用即可复位
};

#endif
#endif
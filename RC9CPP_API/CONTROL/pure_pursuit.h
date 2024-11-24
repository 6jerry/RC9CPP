#ifndef PURE_PURSUIT_H
#define PURE_PURSUIT_H

#ifdef __cplusplus
extern "C"
{
#endif

#include "Vector2D.h"
#include "SuperPID.h"
#include "TrapezoidalPlanner.h"

#ifdef __cplusplus
}
#endif

#ifdef __cplusplus

enum pure_pursuit_mode
{
    normalcontrol, // 法向控制版本，在中低速跟踪较为准确，误差减小快，但追踪速度太高追踪效果就会差
    dircontrol,    // 方向控制版本

    // 切向速度可选的控制模式
    TD_PID,       // 一开始使用TD使得加速平滑，等轨迹快结束了就使用pid精确停在轨迹末尾，较为灵活
    TRAPEZOID_PID // 使用梯形规划和pid，适合一开始就知道整条轨迹和整条轨迹比较固定的情况
};

class pure_pursuit
{
private:
    Vector2D now_dis;

    Vector2D projected;
    Vector2D target_line;
    Vector2D target_line_initpoint;
    Vector2D project_point;
    Vector2D normal_dir;
    Vector2D tangent_dir;
    float normal_dis = 0.0f;
    float tangent_dis = 0.0f;
    float target_dis = 0.0f;
    Vector2D target_wspeed;
    float tan_speed = 0.0f;

    bool if_loop = false;       // 是否是环形轨迹?,如果是环形轨迹的话起始点要写两次
    float tracked_lenth = 0.0f; // 沿着轨迹追踪多远了？
    float change_point = 16.0f; // 啥时候切换点

    pure_pursuit_mode nor_mode = normalcontrol; // 法向纠偏所选择的控制模式

    void compute_error();

    void purepusit_normal_control();

    void purepusit_dir_control();

    Vector2DQueue points_buffer; // 目标点缓冲队列

public:
    Vector2D pursuit(Vector2D now_pos);

    pure_pursuit(pure_pursuit_mode mode, float kp_, float ki_, float kd_, float ktp, float kti, float ktd, float r = 0.0f);

    IncrePID normal_control; // 纯追踪的第一种控制方式，法向纠偏pid

    IncrePID dir_control; // 纯追踪的第二种控制方式，速度矢量方向控制

    bool add_point(Vector2D new_point);
    void force_add_point(Vector2D new_point);

    bool add_points(Vector2D new_points[]);
    void force_add_points(Vector2D new_points[]);

    void refresh_points(); // 清空目标点缓冲队列
};

#endif

#endif
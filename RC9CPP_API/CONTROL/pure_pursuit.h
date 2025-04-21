#ifndef PURE_PURSUIT_H
#define PURE_PURSUIT_H

#ifdef __cplusplus
extern "C"
{
#endif

#include "Vector2D.h"
#include "PID.h"
#include "TrapezoidalPlanner.h"

#ifdef __cplusplus
}
#endif

#ifdef __cplusplus

#define MAX_SPEED 2.2f
#define MAX_NORSPEED 1.86f

class pointrack
{
private:
    float taerget_dis = 0.0f;

public:
    Vector2D track(Vector2D now_pos_, Vector2D target_point_); // 输入当前点和目标点，输出速度矢量

    float get_dis(); // 获取与目标点的距离

    pointrack() {};

    pid track_pid;
};

class yaw_adjuster
{

private:
    void calc_angle_error();
    float target_angle = 0.0f, real_angle = 0.0f, angle_error = 0.0f;

public:
    pid yaw_pid;
    float yaw_adjust(float now_angle, float target_angle_);

    yaw_adjuster() {};
};

enum pure_pursuit_mode
{
    normalcontrol, // 法向控制版本，在中低速跟踪较为准确，误差减小快，但追踪速度太高追踪效果就会差
    dircontrol,    // 方向控制版本，无论何种速度，都超级准确，但流畅度可能不如法向控制版本

    // 切向速度可选的控制模式
    TD_PID,       // 一开始使用TD使得加速平滑，缓冲区里没有点了就会用点追踪停在最后一个点
    TRAPEZOID_PID // 使用梯形规划和pid，适合一开始就知道整条轨迹和整条轨迹比较固定的情况
};

enum pp_state
{
    pp_standby,
    pp_restart,
    pp_tracking,
    pp_stoping
};

class pure_pursuit
{
private:
    Vector2D now_dis, now_point, head, tail;

    Vector2D projected;
    Vector2D target_line;
    Vector2D target_line_initpoint;
    Vector2D project_point;
    Vector2D normal_dir;
    Vector2D tangent_dir;
    Vector2D now_vel;

    float tangent_dis = 0.0f;
    float target_dis = 0.0f;
    Vector2D target_wspeed;
    float tan_speed = 0.0f;

    bool if_loop = false;        // 是否是环形轨迹?,如果是环形轨迹的话起始点要写两次
    float tracked_lenth = 0.0f;  // 沿着轨迹追踪多远了？
    float change_point = 0.004f; // 啥时候切换点

    pure_pursuit_mode nor_mode = normalcontrol; // 法向纠偏所选择的控制模式

    pp_state state = pp_standby;

    void compute_error();

    void purepusit_normal_control();

    void purepusit_dir_control();

    Vector2DQueue points_buffer; // 目标点缓冲队列

public:
    Vector2D pursuit(Vector2D now_pos);
    float normal_dis = 0.0f;
    pure_pursuit() {};

    pid normal_control, tangent_control; // 纯追踪的第一种控制方式，法向纠偏pid

    pid dir_control; // 纯追踪的第二种控制方式，速度矢量方向控制

    TrapezoidalPlanner1D trapezoidal_planner; // 轨迹规划器

    bool pp_add_point(Vector2D new_point);
    void pp_force_add_point(Vector2D new_point);

    bool pp_add_points(Vector2D new_points[], uint8_t length);
    void pp_force_add_points(Vector2D new_points[], uint8_t length);

    void pp_start_plan(Vector2D start_point, Vector2D end_point,Vector2D now_robvel);
    void pp_rst_plan();

    void pp_refresh_points(); // 清空目标点缓冲队列

    float getRemainingDistance();
};

#endif

#endif
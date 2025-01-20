#ifndef TRAPEZOIDAL_PLANNER_H
#define TRAPEZOIDAL_PLANNER_H

#ifdef __cplusplus
extern "C"
{
#endif
#include "arm_math.h"
#ifdef __cplusplus
}
#endif
#ifdef __cplusplus

enum plan_state
{

    plan_standby,
    uniform_acc, // 匀加速
    uniform,     // 匀速
    uniform_dec  // 匀减速

};

enum plan_mode
{
    speed_pulse, // 速度脉冲模式，可能停的不准
    displacement // 位移模式，停的又快又准

};

typedef struct plan_info_
{
    float start_pos = 0.0f;
    float uniform_start_pos = 0.0f;
    float dec_start_pos = 0.0f;
    float acc = 0.0f;
    float dec = 0.0f; // 减速度，取赋值，规划器返回的目标速度都是正的
    float uniform_speed = 0.0f;
    float end_pos = 0.0f;
    float start_speed = 0.0f;
    float end_speed = 0.0f;
};

class TrapezoidalPlanner
{
public:
    bool speed_pulse_plan_start(float max_speed, float slowdown_pos, float acc_, float dec_, float start_pos = 0.0f, float start_speed = 0.0f, float end_speed = 0.0f); // 速度脉冲模式，该模式只关注速度峰值和达到速度峰值的位置，而不关注整体的移动距离
    float speed_pulse_plan_setpos(float pos);                                                                                                                           // 速度脉冲模式下传入实时的位置,返回当前的期望速度

    plan_state get_state(); // 看看当前处在哪一个速度段
    plan_state now_state = plan_standby;
    plan_info_ plan_info;

private:
   
   
    float target_speed = 0.0f, startspeed2 = 0.0f, uniform_speed2 = 0.0f;
};
#endif
#endif // TRAPEZOIDAL_PLANNER_H

#ifndef TRAPEZOIDAL_PLANNER_H
#define TRAPEZOIDAL_PLANNER_H

#include "arm_math.h" // ARM DSP库，用于数学运算
#include "pid.h"      // PID控制器类头文件

class TrapezoidalPlanner
{
public:
    // 构造函数，初始化起始速度、最大速度、结束速度、加速和减速比例以及PID控制器引用
    TrapezoidalPlanner(pid &pidController);

    // 设置新目标参数
    void setTargetParameters(float vStart, float vMax, float vEnd, float accRatio, float decRatio, float totalDistance);

    // 获取当前位置的目标速度
    float computeTargetVelocity(float position);

    // 查询是否到达目标
    bool isTargetReached() const;

    // 获取当前状态名称
    const char *getCurrentStateName() const;

private:
    // 定义状态机阶段
    enum State
    {
        ACCELERATION,  // 加速阶段
        CRUISE,        // 匀速阶段
        DECELERATION,  // 减速阶段
        PID_ADJUST,    // PID微调阶段
        TARGET_REACHED // 到达目标
    } currentState;    // 当前状态

    // 速度规划参数
    float V_start; // 起始速度
    float V_max;   // 最大速度
    float V_end;   // 结束速度
    float R_acc;   // 加速路程比例
    float R_dec;   // 减速路程比例
    float S_total; // 总路程

    float precisionThreshold = 5.0f; // 进入PID微调的距离阈值
    pid &pidControl;                 // 引用PID控制器以用于微调速度

    // 更新当前状态
    void updateState(float position);
};

#endif // TRAPEZOIDAL_PLANNER_H

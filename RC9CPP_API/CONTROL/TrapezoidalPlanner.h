#ifndef TRAPEZOIDAL_PLANNER_H
#define TRAPEZOIDAL_PLANNER_H

#ifdef __cplusplus
extern "C"
{
#endif
#include "arm_math.h"
#include "Vector2D.h"
//#include "pure_pursuit.h"

#ifdef __cplusplus
}
#endif
#ifdef __cplusplus

// 定义运动阶段枚举，新增 PID_PHASE 表示进入 PID 点追踪控制
enum Phase
{
    ACCEL_PHASE,   // 加速段
    CONST_PHASE,   // 匀速段
    DECEL_PHASE,   // 减速段
    PID_PHASE,     // PID 控制段（接近目标点）
    FINISHED_PHASE // 规划结束
};

// 定义规划类型枚举
enum ProfileType
{
    TRAPEZOIDAL, // 梯形规划：存在加速、匀速、减速三个阶段
    TRIANGULAR   // 三角形规划：仅有加速和减速两个阶段，无法达到设定的最大速度
};

class TrapezoidalPlanner
{
public:
    // 构造函数，初始状态为 FINISHED（无规划）
    TrapezoidalPlanner();

    /**
     * @brief 初始化一次新的规划，重置内部状态和参数
     * @param maxAcc       最大加速度（正值）
     * @param maxDec       最大减速度（正值）
     * @param maxSpeed     最大允许速度
     * @param initialSpeed 起始时的速度
     * @param finalSpeed   目标点的速度
     * @param startPos     起始坐标（Vector2D 类型）
     * @param targetPos    目标坐标（Vector2D 类型）
     * @param pidThreshold 用户设置的 PID 控制距离阈值，若为 0 则不使用 PID 控制
     */
    void start_plan(float maxAcc, float maxDec, float maxSpeed, float initialSpeed, float finalSpeed,
                    const Vector2D &startPos, const Vector2D &targetPos, float pidThreshold = 0.0f);

    /**
     * @brief 根据当前坐标规划目标速度向量（x 和 y 分量）
     * @param currentPos 当前坐标（Vector2D 类型）
     * @return 目标速度向量（Vector2D 类型）；若规划结束则返回终点速度方向
     */
    Vector2D plan(const Vector2D &currentPos);

    // 辅助接口：根据当前已行驶距离判断处于哪个阶段（不含 PID 判断）
    Phase determinePhase(float traveled);

    // 获取当前阶段（调试或外部查询）
    Phase getPhase() const { return m_phase; }
    bool isFinished() const { return m_phase == FINISHED_PHASE; }

private:
    // 内部状态
    Phase m_phase;
    ProfileType m_profileType;

    // 规划参数
    float m_maxAcc;       // 最大加速度
    float m_maxDec;       // 最大减速度
    float m_maxSpeed;     // 最大允许速度
    float m_initialSpeed; // 起始速度
    float m_finalSpeed;   // 目标点速度

    // 路径信息
    Vector2D m_startPos;   // 起始坐标
    Vector2D m_targetPos;  // 目标坐标
    float m_totalDistance; // 总路程

    // 各阶段路程
    float m_accelDistance; // 加速段长度
    float m_decelDistance; // 减速段长度
};

class TrapezoidalPlanner1D
{
public:
    // 构造函数，初始状态为 FINISHED（无规划）
    TrapezoidalPlanner1D();

    /**
     * @brief 初始化一次新的规划，重置内部状态和参数
     * @param maxAcc       最大加速度（正值）
     * @param maxDec       最大减速度（正值）
     * @param maxSpeed     最大允许速度
     * @param initialSpeed 起始时的速度
     * @param finalSpeed   目标点的速度
     * @param startPos     起始位置
     * @param targetPos    目标位置
     */
    void start_plan(float maxAcc, float maxDec, float maxSpeed, float initialSpeed, float finalSpeed, float startPos, float targetPos);

    /**
     * @brief 根据当前已行驶的距离，规划目标速度
     * @param traveled 已行驶的距离
     * @return 目标速度
     */
    float plan(float now_dis);

    float traveled = 0.0f;

    // 辅助接口：根据当前已行驶距离判断处于哪个阶段
    Phase determinePhase(float traveled);

    // 获取当前阶段
    Phase getPhase() const { return m_phase; }

    bool isFinished() const { return m_phase == FINISHED_PHASE; }

    void reset();

public:
    // 内部状态
    Phase m_phase;
    // 规划参数
    float m_maxAcc;        // 最大加速度
    float m_maxDec;        // 最大减速度
    float m_maxSpeed;      // 最大速度
    float m_initialSpeed;  // 起始速度
    float m_finalSpeed;    // 目标速度
    float m_startPos;      // 起始位置
    float m_targetPos;     // 目标位置
    float m_totalDistance; // 总路程

    // 各阶段路程
    float m_accelDistance; // 加速段长度
    float m_decelDistance; // 减速段长度

    float direction = 0.0f;

    float min_dead_speed = 0.0f;
    float v_target = 0.0f;
};

class VelocityPlanner
{
public:
    // 构造函数，传入加速度上限，单位 m/s²
    VelocityPlanner(float maxAcceleration = 0.0f);

    // 规划函数：传入当前目标速度，返回经过时间补偿后的平滑输出速度
    float plan(float targetSpeed);

    // 设置新的加速度上限，单位 m/s²
    void setMaxAcceleration(float acceleration);

    // 重置规划器状态，可设置初始速度，单位 m/s
    void reset(float initialValue = 0.0f, float maxAcceleration = 0.0f);

    void reset_speed();

private:
    float maxAcceleration; // 加速度上限（单位 m/s²，后缀平方秒）
    float lastOutput;      // 上一次输出的速度（单位 m/s）
};

#endif
#endif

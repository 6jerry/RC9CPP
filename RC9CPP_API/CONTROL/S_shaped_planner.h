#ifndef S_SHAPED_PLANNER_H
#define S_SHAPED_PLANNER_H

#ifdef __cplusplus
extern "C"
{
#endif
#include "arm_math.h"  // 使用 ARM CMSIS DSP 库
#include "Vector2D.h"  // 有一个 Vector2D 类或结构体来处理二维向量

#ifdef __cplusplus
}
#endif

#ifdef __cplusplus

// 定义运动阶段枚举
enum SPhase
{
    S_ACCEL_JERK_UP_PHASE,   // 加速段：加加速度（Jerk）从0增加到最大值
    S_ACCEL_CONST_PHASE,     // 加速段：加速度保持恒定在最大值
    S_ACCEL_JERK_DOWN_PHASE, // 加速段：加加速度（Jerk）从最大值减小到0
    S_CONST_VEL_PHASE,       // 匀速段：速度保持恒定在最大值
    S_DECEL_JERK_UP_PHASE,   // 减速段：加加速度（Jerk）从0增加到最大负值（开始减速）
    S_DECEL_CONST_PHASE,     // 减速段：加速度保持恒定在最大负值
    S_DECEL_JERK_DOWN_PHASE, // 减速段：加加速度（Jerk）从最大负值减小到0（减速结束）
    S_FINISHED_PHASE         // 规划完成阶段
};

class SShapedPlanner
{
public:
    // 构造函数：初始化所有成员变量为零或默认值
    SShapedPlanner();

    /**
     * @brief 初始化一次新的 S 型规划配置文件。
     * @param maxAcc 最大加速度（正值）。
     * @param maxDec 最大减速度（正值）。
     * @param maxJerk 最大加加速度（Jerk，正值）。
     * @param maxSpeed 最大允许速度。
     * @param initialSpeed 起始速度。
     * @param finalSpeed 目标速度（终点速度）。
     * @param startPos 起始二维位置。
     * @param targetPos 目标二维位置。
     */
    void start_plan(float maxAcc, float maxDec, float maxJerk, float maxSpeed, float initialSpeed, float finalSpeed,
                    const Vector2D &startPos, const Vector2D &targetPos);

    /**
     * @brief 根据当前位置计算目标速度向量。
     * 此函数通过比较当前位置与规划路径上的预计算距离来确定当前所处的规划阶段，
     * 并根据该阶段返回对应的目标速度。
     * @param currentPos 当前二维位置。
     * @return 目标速度向量。
     */
    Vector2D plan(const Vector2D &currentPos);

    /**
     * @brief 获取当前规划所处的阶段。
     * @return 当前 S 型规划阶段。
     */
    SPhase getPhase() const { return m_phase; }

    /**
     * @brief 判断规划是否已完成。
     * @return 如果规划已完成则返回 true，否则返回 false。
     */
    bool isFinished() const { return m_phase == S_FINISHED_PHASE; }

private:
    // 内部状态变量
    SPhase m_phase;         // 当前规划所处的阶段

    // 规划参数
    float m_maxAcc;       // 最大加速度
    float m_maxDec;       // 最大减速度
    float m_maxJerk;      // 最大加加速度（Jerk）
    float m_maxSpeed;     // 最大允许速度
    float m_initialSpeed; // 起始速度
    float m_finalSpeed;   // 终点速度

    // 路径信息
    Vector2D m_startPos;   // 起始位置
    Vector2D m_targetPos;  // 目标位置
    float m_totalDistance; // 总路程（从起始位置到目标位置的直线距离）

    // 预计算的 S 型规划各个阶段的距离。
    // 在 `start_plan` 中计算这些距离，以便在 `plan` 函数中快速判断当前阶段。
    // 对于一个完整的 S 型曲线，这些距离的精确计算需要复杂的运动学公式。
    float m_accelJerkUpDistance;   // 加速段：Jerk 上升阶段的路程
    float m_accelConstDistance;    // 加速段：加速度恒定阶段的路程
    float m_accelJerkDownDistance; // 加速段：Jerk 下降阶段的路程
    float m_constVelDistance;      // 匀速段：恒定速度阶段的路程
    float m_decelJerkUpDistance;   // 减速段：Jerk 上升（减速开始）阶段的路程
    float m_decelConstDistance;    // 减速段：加速度恒定（减速中）阶段的路程
    float m_decelJerkDownDistance; // 减速段：Jerk 下降（减速结束）阶段的路程

    /**
     * @brief 根据已行驶的距离确定当前所处的 S 型规划阶段。
     * @param traveled 已行驶的距离（从起始位置算起）。
     * @return 当前 S 型规划阶段。
     */
    SPhase determinePhase(float traveled);

    /**
     * @brief 预计算各阶段的距离
     */
    void cal_PhaseDistances();

    /**
     * @brief 加速段：Jerk 上升阶段的速度
     * @param traveled 已行驶距离
     * @return 当前速度
     */
    float cal_Acc_JerkUpSpeed(float traveled);

    /**
     * @brief 加速段：加速度恒定阶段的速度
     * @param traveled 已行驶距离
     * @return 当前速度
     */
    float cal_Acc_ConstSpeed(float traveled);

    /**
     * @brief 加速段：Jerk 下降阶段的速度
     * @param traveled 已行驶距离
     * @return 当前速度
     */
    float cal_Acc_JerkDownSpeed(float traveled);

    /**
     * @brief 减速段：Jerk 上升阶段的速度
     * @param traveled 已行驶距离
     * @return 当前速度
     */
    float cal_Dec_JerkUpSpeed(float traveled);

    /**
     * @brief 减速段：加速度恒定阶段的速度
     * @param traveled 已行驶距离
     * @return 当前速度
     */
    float cal_Dec_ConstSpeed(float traveled);

    /**
     * @brief 减速段：Jerk 下降阶段的速度
     * @param traveled 已行驶距离
     * @return 当前速度
     */
    float cal_Dec_JerkDownSpeed(float traveled);
};

#endif // __cplusplus

#endif // S_SHAPED_PLANNER_H
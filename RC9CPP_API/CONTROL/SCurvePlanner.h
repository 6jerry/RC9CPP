#ifndef SCURVE_PLANNER_H
#define SCURVE_PLANNER_H

#include "TrapezoidalPlanner.h"

class SCurvePlanner : public TrapezoidalPlanner {
public:
    SCurvePlanner();
    
    /**
     * @brief 设置S型曲线参数
     * @param jerk 加加速度 (m/s³)
     * @param maxAcc 最大加速度 (m/s²)
     * @param maxDec 最大减速度 (m/s²)
     */
    void setScurveParams(float jerk, float maxAcc, float maxDec);
    
    /**
     * @brief 开始S型规划
     * @param maxSpeed 最大速度 (m/s)
     * @param initialSpeed 初始速度 (m/s)
     * @param finalSpeed 最终速度 (m/s)
     * @param startPos 起始位置 (Vector2D)
     * @param targetPos 目标位置 (Vector2D)
     */
    void startScurve(float maxSpeed, float initialSpeed, float finalSpeed,
                    const Vector2D &startPos, const Vector2D &targetPos);
    
    /**
     * @brief 计算当前目标速度
     * @param currentPos 当前位置
     * @return 目标速度向量
     */
    Vector2D plan(const Vector2D &currentPos) override;

private:
    // S型曲线阶段
    enum ScurvePhase {
        ACCEL_RAMP_UP,   // 加速上升段
        ACCEL_CONST,     // 加速保持段
        ACCEL_RAMP_DOWN, // 加速下降段
        DECEL_RAMP_UP,   // 减速上升段
        DECEL_CONST,     // 减速保持段
        DECEL_RAMP_DOWN  // 减速下降段
    };
    
    // S型曲线参数
    float m_jerk = 0.0f;    // 加加速度 (m/s³)
    float m_maxAcc = 0.0f;  // 最大加速度 (m/s²)
    float m_maxDec = 0.0f;  // 最大减速度 (m/s²)
    
    // 当前阶段
    ScurvePhase m_currentPhase = ACCEL_RAMP_UP;
    
    // 阶段计时器
    float m_phaseTime = 0.0f;
    
    // 计算阶段时间
    float calculatePhaseTime(float startSpeed, float endSpeed, float acc) const;
};
#endif
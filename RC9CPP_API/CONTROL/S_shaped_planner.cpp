#include "S_shaped_planner.h"
#include <cmath>  // 用于数学运算，如 sqrt, pow 等

// 构造函数：初始化所有成员变量为零或默认值
SShapedPlanner::SShapedPlanner()
    : m_phase(S_ACCEL_JERK_UP_PHASE),
      m_maxAcc(0.0f), m_maxDec(0.0f), m_maxJerk(0.0f),
      m_maxSpeed(0.0f), m_initialSpeed(0.0f), m_finalSpeed(0.0f),
      m_totalDistance(0.0f),
      m_accelJerkUpDistance(0.0f), m_accelConstDistance(0.0f), m_accelJerkDownDistance(0.0f),
      m_constVelDistance(0.0f),
      m_decelJerkUpDistance(0.0f), m_decelConstDistance(0.0f), m_decelJerkDownDistance(0.0f)
{
    // 空实现
}

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
void SShapedPlanner::start_plan(float maxAcc, float maxDec, float maxJerk, float maxSpeed,
                                float initialSpeed, float finalSpeed,
                                const Vector2D &startPos, const Vector2D &targetPos)
{
    // 保存规划参数
    m_maxAcc = maxAcc;
    m_maxDec = maxDec;
    m_maxJerk = maxJerk;
    m_maxSpeed = maxSpeed;
    m_initialSpeed = initialSpeed;
    m_finalSpeed = finalSpeed;
    m_startPos = startPos;
    m_targetPos = targetPos;

    // 计算总距离
    Vector2D delta = targetPos - startPos;
    m_totalDistance = delta.magnitude();

    // 预计算各个阶段的路程
    cal_PhaseDistances();

    // 初始化当前阶段
    m_phase = S_ACCEL_JERK_UP_PHASE;
}

/**
 * @brief 根据当前位置计算目标速度向量。
 * 此函数通过比较当前位置与规划路径上的预计算距离来确定当前所处的规划阶段，
 * 并根据该阶段返回对应的目标速度。
 * @param currentPos 当前二维位置。
 * @return 目标速度向量。
 */
Vector2D SShapedPlanner::plan(const Vector2D &currentPos)
{
    // 计算已行驶的距离
    Vector2D traveledVec = currentPos - m_startPos;
    float traveled = traveledVec.magnitude();

    // 确定当前阶段
    m_phase = determinePhase(traveled);

    // 根据阶段计算当前速度
    float currentSpeed = 0.0f;

    switch (m_phase)
    {
        case S_ACCEL_JERK_UP_PHASE:
            currentSpeed = cal_Acc_JerkUpSpeed(traveled);
            break;
        case S_ACCEL_CONST_PHASE:
            currentSpeed = cal_Acc_ConstSpeed(traveled);
            break;
        case S_ACCEL_JERK_DOWN_PHASE:
            currentSpeed = cal_Acc_JerkDownSpeed(traveled);
            break;
        case S_CONST_VEL_PHASE:
            currentSpeed = m_maxSpeed;
            break;
        case S_DECEL_JERK_UP_PHASE:
            currentSpeed = cal_Dec_JerkUpSpeed(traveled);
            break;
        case S_DECEL_CONST_PHASE:
            currentSpeed = cal_Dec_ConstSpeed(traveled);
            break;
        case S_DECEL_JERK_DOWN_PHASE:
            currentSpeed = cal_Dec_JerkDownSpeed(traveled);
            break;
        case S_FINISHED_PHASE:
            currentSpeed = m_finalSpeed;
            break;
    }

    // 返回速度向量（方向与路径一致）
    Vector2D direction = (m_targetPos - m_startPos).normalize();
    return direction * currentSpeed;
}

/**
 * @brief 根据已行驶的距离确定当前所处的 S 型规划阶段。
 * @param traveled 已行驶的距离（从起始位置算起）。
 * @return 当前 S 型规划阶段。
 */
SPhase SShapedPlanner::determinePhase(float traveled)
{
    // 检查是否已经完成
    if (traveled >= m_totalDistance)
    {
        return S_FINISHED_PHASE;
    }

    // 累计距离判断当前阶段
    float cumulative = 0.0f;

    // 加速段：Jerk 上升
    cumulative += m_accelJerkUpDistance;
    if (traveled < cumulative)
        return S_ACCEL_JERK_UP_PHASE;

    // 加速段：加速度恒定
    cumulative += m_accelConstDistance;
    if (traveled < cumulative)
        return S_ACCEL_CONST_PHASE;

    // 加速段：Jerk 下降
    cumulative += m_accelJerkDownDistance;
    if (traveled < cumulative)
        return S_ACCEL_JERK_DOWN_PHASE;

    // 匀速段
    cumulative += m_constVelDistance;
    if (traveled < cumulative)
        return S_CONST_VEL_PHASE;

    // 减速段：Jerk 上升
    cumulative += m_decelJerkUpDistance;
    if (traveled < cumulative)
        return S_DECEL_JERK_UP_PHASE;

    // 减速段：加速度恒定
    cumulative += m_decelConstDistance;
    if (traveled < cumulative)
        return S_DECEL_CONST_PHASE;

    // 减速段：Jerk 下降
    return S_DECEL_JERK_DOWN_PHASE;
}

// ---------------------------- 内部辅助函数 ----------------------------

/**
 * @brief 计算各阶段的距离
 */
void SShapedPlanner::cal_PhaseDistances()
{
    // 先计算加速阶段的 Jerk 时间
    float t_jerk_up = m_maxAcc / m_maxJerk;
    float t_jerk_up_2 = t_jerk_up * t_jerk_up;
    float t_jerk_up_3 = t_jerk_up_2 * t_jerk_up;
    m_accelJerkUpDistance = m_initialSpeed * t_jerk_up + 0.5f * m_maxJerk * t_jerk_up_3 / 3.0f;

    // 加速段：加速度保持恒定的时间
    float speed_after_jerk_up = m_initialSpeed + 0.5f * m_maxJerk * t_jerk_up_2;
    float time_accel_const = (m_maxSpeed - speed_after_jerk_up) / m_maxAcc;
    float time_accel_const_2 = time_accel_const * time_accel_const;
    m_accelConstDistance = speed_after_jerk_up * time_accel_const + 0.5f * m_maxAcc * time_accel_const_2;

    // Jerk 下降阶段（加速度回到 0）
    float t_jerk_down = m_maxAcc / m_maxJerk;
    float t_jerk_down_2 = t_jerk_down * t_jerk_down;
    float t_jerk_down_3 = t_jerk_down_2 * t_jerk_down;
    float speed_after_accel_const = speed_after_jerk_up + m_maxAcc * time_accel_const;
    m_accelJerkDownDistance = speed_after_accel_const * t_jerk_down - (m_maxJerk * t_jerk_down_3) / 6.0f;

    // 匀速阶段距离
    float speed_during_const = speed_after_accel_const + 0.5f * m_maxJerk * t_jerk_down_2;
    float distance_remaining = m_totalDistance - (m_accelJerkUpDistance + m_accelConstDistance + m_accelJerkDownDistance);
    m_constVelDistance = distance_remaining * (speed_during_const >= m_maxSpeed ? 1.0f : 0.0f);

    // 减速阶段（与加速阶段对称）
    float decel_time_jerk_up = m_maxDec / m_maxJerk;
    float decel_time_jerk_up_2 = decel_time_jerk_up * decel_time_jerk_up;
    float decel_time_jerk_up_3 = decel_time_jerk_up_2 * decel_time_jerk_up;
    m_decelJerkUpDistance = m_maxSpeed * decel_time_jerk_up - 0.5f * m_maxJerk * decel_time_jerk_up_3 / 3.0f;

    float speed_after_decel_jerk_up = m_maxSpeed - 0.5f * m_maxJerk * decel_time_jerk_up_2;
    float decel_time_const = (speed_after_decel_jerk_up - m_finalSpeed) / m_maxDec;
    float decel_time_const_2 = decel_time_const * decel_time_const;
    m_decelConstDistance = speed_after_decel_jerk_up * decel_time_const - 0.5f * m_maxDec * decel_time_const_2;

    float decel_time_jerk_down = m_maxDec / m_maxJerk;
    float decel_time_jerk_down_2 = decel_time_jerk_down * decel_time_jerk_down;
    float decel_time_jerk_down_3 = decel_time_jerk_down_2 * decel_time_jerk_down;
    float speed_after_decel_const = speed_after_decel_jerk_up - m_maxDec * decel_time_const;
    m_decelJerkDownDistance = speed_after_decel_const * decel_time_jerk_down - (m_maxJerk * decel_time_jerk_down_3) / 6.0f;
}

/**
 * @brief 加速段：Jerk 上升阶段的速度
 */
float SShapedPlanner::cal_Acc_JerkUpSpeed(float traveled)
{
    float t;
    arm_sqrt_f32(2.0f * traveled / m_maxJerk, &t);
    return m_initialSpeed + 0.5f * m_maxJerk * t * t;
}

/**
 * @brief 加速段：加速度恒定阶段的速度
 */
float SShapedPlanner::cal_Acc_ConstSpeed(float traveled)
{
    float t = (traveled - m_accelJerkUpDistance) / m_maxAcc;
    return m_initialSpeed + m_maxAcc * t;
}

/**
 * @brief 加速段：Jerk 下降阶段的速度
 */
float SShapedPlanner::cal_Acc_JerkDownSpeed(float traveled)
{
    float distance = traveled - m_accelJerkUpDistance - m_accelConstDistance;
    float t;
    arm_sqrt_f32(2.0f * distance / m_maxJerk, &t);
    return m_maxSpeed - 0.5f * m_maxJerk * t * t;
}

/**
 * @brief 减速段：Jerk 上升阶段的速度
 */
float SShapedPlanner::cal_Dec_JerkUpSpeed(float traveled)
{
    float remaining = m_totalDistance - traveled;
    float t;
    arm_sqrt_f32(2.0f * remaining / m_maxJerk, &t);
    return m_maxSpeed - 0.5f * m_maxJerk * t * t;
}

/**
 * @brief 减速段：加速度恒定阶段的速度
 */
float SShapedPlanner::cal_Dec_ConstSpeed(float traveled)
{
    float remaining = m_totalDistance - traveled;
    float t = remaining / m_maxDec;
    return m_maxSpeed - m_maxDec * t;
}

/**
 * @brief 减速段：Jerk 下降阶段的速度
 */
float SShapedPlanner::cal_Dec_JerkDownSpeed(float traveled)
{
    float remaining = m_totalDistance - traveled;
    float t;
    arm_sqrt_f32(2.0f * remaining / m_maxJerk, &t);
    return m_finalSpeed + 0.5f * m_maxJerk * t * t;
}
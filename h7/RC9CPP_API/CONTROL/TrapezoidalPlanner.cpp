#include "TrapezoidalPlanner.h"
TrapezoidalPlanner::TrapezoidalPlanner()
    : m_phase(FINISHED_PHASE), m_profileType(TRAPEZOIDAL),
      m_maxAcc(0), m_maxDec(0), m_maxSpeed(0),
      m_initialSpeed(0), m_finalSpeed(0), m_totalDistance(0),
      m_accelDistance(0), m_decelDistance(0)
{}



void TrapezoidalPlanner::start_plan(float maxAcc, float maxDec, float maxSpeed, float initialSpeed, float finalSpeed,
                                    const Vector2D &startPos, const Vector2D &targetPos, float pidThreshold)
{
    // 保存用户参数
    m_maxAcc = abs(maxAcc);
    m_maxDec = abs(maxDec);
    m_maxSpeed = abs(maxSpeed);
    m_initialSpeed = abs(initialSpeed);
    m_finalSpeed = abs(finalSpeed);
    m_startPos = startPos;
    m_targetPos = targetPos;

    // 计算总路程：起点到目标点的直线距离
    Vector2D diff = m_targetPos - m_startPos;
    m_totalDistance = diff.magnitude();

    // 计算若能达到设定最大速度时的加速和减速路程
    float d_acc = 0;
    if (m_maxSpeed > m_initialSpeed)
        d_acc = (m_maxSpeed * m_maxSpeed - m_initialSpeed * m_initialSpeed) / (2.0f * m_maxAcc);
    float d_dec = 0;
    if (m_maxSpeed > m_finalSpeed)
        d_dec = (m_maxSpeed * m_maxSpeed - m_finalSpeed * m_finalSpeed) / (2.0f * m_maxDec);

    // 判断是否能够达到设定最大速度
    if (d_acc + d_dec <= m_totalDistance)
    {
        // 梯形规划：存在加速、匀速、减速三个阶段
        m_profileType = TRAPEZOIDAL;
        m_accelDistance = d_acc;
        m_decelDistance = d_dec;
    }
    else
    {
        // 三角形规划：无法达到设定最大速度，计算可达到的峰值速度 v_peak
        m_profileType = TRIANGULAR;
        float v_peak_sq = (m_maxDec * m_initialSpeed * m_initialSpeed +
                           m_maxAcc * m_finalSpeed * m_finalSpeed +
                           2 * m_maxAcc * m_maxDec * m_totalDistance) /
                          (m_maxAcc + m_maxDec);
        float v_peak = 0.0f;
        arm_sqrt_f32(v_peak_sq, &v_peak);
        m_accelDistance = (v_peak * v_peak - m_initialSpeed * m_initialSpeed) / (2.0f * m_maxAcc);
        m_decelDistance = (v_peak * v_peak - m_finalSpeed * m_finalSpeed) / (2.0f * m_maxDec);
    }

    // 初始化阶段为加速段
    m_phase = ACCEL_PHASE;
}

Phase TrapezoidalPlanner::determinePhase(float traveled)
{
    if (traveled >= m_totalDistance)
        return FINISHED_PHASE;

    if (m_profileType == TRAPEZOIDAL)
    {
        if (traveled < m_accelDistance)
            return ACCEL_PHASE;
        else if (traveled < (m_totalDistance - m_decelDistance))
            return CONST_PHASE;
        else
            return DECEL_PHASE;
    }
    else
    { // TRIANGULAR
        if (traveled < m_accelDistance)
            return ACCEL_PHASE;
        else
            return DECEL_PHASE;
    }
}

Vector2D TrapezoidalPlanner::plan(const Vector2D &currentPos)
{
    // 计算路径及单位方向
    Vector2D path = m_targetPos - currentPos;
    if (m_totalDistance < 0.0001f)
    {
        m_phase = FINISHED_PHASE;
        return Vector2D(0, 0);
    }
    Vector2D direction = path.normalize();

    // 计算当前位置在规划路径上的投影距离
    Vector2D delta = currentPos - m_startPos;
    float traveled = delta * direction;
    if (traveled < 0)
        traveled = 0;
    if (traveled >= m_totalDistance)
    {
        return m_finalSpeed * (m_targetPos - m_startPos).normalize();
    }
    // traveled = m_totalDistance;

    // 计算当前位置与目标点之间的直线距离
    float distanceToTarget = (m_targetPos - currentPos).magnitude();

    // 未进入 PID 控制则继续采用梯形规划，根据 traveled 判断当前阶段
    m_phase = determinePhase(traveled);
    float v_target = 0;
    switch (m_phase)
    {
    case ACCEL_PHASE:
    {
        float expr = m_initialSpeed * m_initialSpeed + 2 * m_maxAcc * traveled;
        float sqrt_val = 0;
        arm_sqrt_f32(expr, &sqrt_val);
        v_target = sqrt_val;
        break;
    }
    case CONST_PHASE:
        v_target = m_maxSpeed;
        break;
    case DECEL_PHASE:
    {
        float expr = m_finalSpeed * m_finalSpeed + 2 * m_maxDec * (m_totalDistance - traveled);
        float sqrt_val = 0;
        arm_sqrt_f32(expr, &sqrt_val);
        v_target = sqrt_val;
        break;
    }
    case FINISHED_PHASE:
    default:
        v_target = m_finalSpeed;
        break;
    }

 
    return direction * v_target;
}

TrapezoidalPlanner1D::TrapezoidalPlanner1D()
    : m_phase(FINISHED_PHASE), m_maxAcc(0), m_maxDec(0), m_maxSpeed(0),
      m_initialSpeed(0), m_finalSpeed(0), m_totalDistance(0),
      m_accelDistance(0), m_decelDistance(0)
{
}

void TrapezoidalPlanner1D::start_plan(float maxAcc, float maxDec, float maxSpeed, float initialSpeed, float finalSpeed, float startPos, float targetPos)
{
    // 保存用户参数
    m_maxAcc = abs(maxAcc);
    m_maxDec = abs(maxDec);
    m_maxSpeed = abs(maxSpeed);
    m_initialSpeed = abs(initialSpeed);
    m_finalSpeed = abs(finalSpeed);
    m_startPos = startPos;
    m_targetPos = targetPos;

    // 计算总路程
    m_totalDistance = abs(targetPos - startPos);

    if (targetPos - startPos > 0.0f)
    {
        direction = 1.0f;
    }
    else if (targetPos - startPos < 0.0f)
    {
        direction = -1.0f;
    }

    // 计算加速和减速所需的路程
    float d_acc = (m_maxSpeed * m_maxSpeed - m_initialSpeed * m_initialSpeed) / (2.0f * m_maxAcc);
    float d_dec = (m_maxSpeed * m_maxSpeed - m_finalSpeed * m_finalSpeed) / (2.0f * m_maxDec);

    // 判断是否能够达到设定最大速度
    if (d_acc + d_dec <= m_totalDistance)
    {
        // 梯形规划：存在加速、匀速、减速三个阶段
        m_accelDistance = d_acc;
        m_decelDistance = d_dec;
    }
    else
    {
        // 三角形规划：无法达到设定最大速度，计算可达到的峰值速度 v_peak
        float v_peak_sq = (m_maxDec * m_initialSpeed * m_initialSpeed +
                           m_maxAcc * m_finalSpeed * m_finalSpeed +
                           2 * m_maxAcc * m_maxDec * m_totalDistance) /
                          (m_maxAcc + m_maxDec);
        float v_peak = 0.0f;
        arm_sqrt_f32(v_peak_sq, &v_peak);
        m_accelDistance = (v_peak * v_peak - m_initialSpeed * m_initialSpeed) / (2.0f * m_maxAcc);
        m_decelDistance = (v_peak * v_peak - m_finalSpeed * m_finalSpeed) / (2.0f * m_maxDec);
    }

    // 初始化阶段为加速段
    m_phase = ACCEL_PHASE;
}

Phase TrapezoidalPlanner1D::determinePhase(float traveled)
{
    if (traveled >= m_totalDistance)
        return FINISHED_PHASE;

    if (traveled < m_accelDistance)
        return ACCEL_PHASE;
    else if (traveled < (m_totalDistance - m_decelDistance))
        return CONST_PHASE;
    else
        return DECEL_PHASE;
}

float TrapezoidalPlanner1D::plan(float now_dis)
{

    traveled = abs(now_dis - m_startPos);
    if (traveled >= m_totalDistance)
    {
        traveled = m_totalDistance;
        // m_phase = FINISHED_PHASE;
        return m_finalSpeed * direction;
    }

    // 判断当前阶段
    m_phase = determinePhase(traveled);

    switch (m_phase)
    {
    case ACCEL_PHASE:
    {
        float expr = m_initialSpeed * m_initialSpeed + 2.0f * m_maxAcc * traveled;
        float sqrt_val = 0.0f;
        arm_sqrt_f32(expr, &sqrt_val);
        v_target = sqrt_val;

        break;
    }
    case CONST_PHASE:
        v_target = m_maxSpeed;
        break;
    case DECEL_PHASE:
    {
        float expr = m_finalSpeed * m_finalSpeed + 2 * m_maxDec * (m_totalDistance - traveled);
        float sqrt_val = 0;
        arm_sqrt_f32(expr, &sqrt_val);
        v_target = sqrt_val;
        break;
    }
    case FINISHED_PHASE:
    default:
        v_target = m_finalSpeed;
        break;
    }

    return v_target * direction;
}

void TrapezoidalPlanner1D::reset()
{
    m_phase = FINISHED_PHASE;
    m_totalDistance = 0;
    m_accelDistance = 0;
    m_decelDistance = 0;
    m_totalDistance = 0;
    direction = 0;
}

VelocityPlanner::VelocityPlanner(float maxAcceleration)
{
    this->maxAcceleration = maxAcceleration;
    lastOutput = 0.0f;
}

float VelocityPlanner::plan(float targetSpeed)
{
    float diff = targetSpeed - lastOutput;
    // 如果目标速度与上一次规划的速度差超过上限，则限制增量
    if (fabs(diff) > maxAcceleration)
    {
        if (diff > 0)
        {
            diff = maxAcceleration;
        }
        else
        {
            diff = -maxAcceleration;
        }
    }
    lastOutput += diff;
    return lastOutput;
}

void VelocityPlanner::setMaxAcceleration(float acceleration)
{
    maxAcceleration = acceleration;
}

void VelocityPlanner::reset(float initialValue, float maxAcceleration)
{
    lastOutput = initialValue;

    this->maxAcceleration = maxAcceleration;
}

void VelocityPlanner::reset_speed()
{
    lastOutput = 0.0f;
}
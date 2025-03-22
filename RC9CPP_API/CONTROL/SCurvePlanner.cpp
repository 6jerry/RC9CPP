#include "SCurvePlanner.h"

SCurvePlanner::SCurvePlanner() {
    m_profileType = S_CURVE;
}

void SCurvePlanner::setScurveParams(float jerk, float maxAcc, float maxDec) {
    m_jerk = jerk;
    m_maxAcc = maxAcc;
    m_maxDec = maxDec;
}

void SCurvePlanner::startScurve(float maxSpeed, float initialSpeed, float finalSpeed,
                               const Vector2D &startPos, const Vector2D &targetPos) {
    start_plan(m_maxAcc, m_maxDec, maxSpeed, initialSpeed, finalSpeed, startPos, targetPos);
    m_currentPhase = ACCEL_RAMP_UP;
    m_phaseTime = 0.0f;
}

Vector2D SCurvePlanner::plan(const Vector2D &currentPos) {
    static float last_m_phaseTime = 0;
    m_phaseTime = HAL_GetTick();
    // 计算路径方向
    Vector2D path = m_targetPos - m_startPos;
    Vector2D direction = path.normalize();
    
    // 计算已行驶距离
    Vector2D delta = currentPos - m_startPos;
    float traveled = delta * direction;
    
    // 计算目标速度
    float v_target = 0.0f;
    switch (m_currentPhase) {
        case ACCEL_RAMP_UP:
            v_target = m_initialSpeed + 0.5f * m_jerk * m_phaseTime * m_phaseTime;
            if (v_target >= m_maxSpeed) {
                m_currentPhase = ACCEL_CONST;
                m_phaseTime = 0.0f;
            }
            break;
            
        case ACCEL_CONST:
            v_target = m_maxSpeed;
            if (traveled >= m_accelDistance) {
                m_currentPhase = ACCEL_RAMP_DOWN;
                m_phaseTime = 0.0f;
            }
            break;
            
        case ACCEL_RAMP_DOWN:
            v_target = m_maxSpeed - 0.5f * m_jerk * m_phaseTime * m_phaseTime;
            if (v_target <= m_maxSpeed - m_maxAcc) {
                m_currentPhase = CONST_PHASE;
                m_phaseTime = 0.0f;
            }
            break;
            
        case CONST_PHASE:
            v_target = m_maxSpeed;
            if (traveled >= m_totalDistance - m_decelDistance) {
                m_currentPhase = DECEL_RAMP_UP;
                m_phaseTime = 0.0f;
            }
            break;
            
        case DECEL_RAMP_UP:
            v_target = m_maxSpeed - 0.5f * m_jerk * m_phaseTime * m_phaseTime;
            if (v_target <= m_maxSpeed - m_maxDec) {
                m_currentPhase = DECEL_CONST;
                m_phaseTime = 0.0f;
            }
            break;
            
        case DECEL_CONST:
            v_target = m_maxSpeed - m_maxDec;
            if (traveled >= m_totalDistance - m_decelDistance) {
                m_currentPhase = DECEL_RAMP_DOWN;
                m_phaseTime = 0.0f;
            }
            break;
            
        case DECEL_RAMP_DOWN:
            v_target = m_finalSpeed + 0.5f * m_jerk * m_phaseTime * m_phaseTime;
            if (v_target >= m_finalSpeed) {
                m_currentPhase = FINISHED_PHASE;
            }
            break;
            
        case FINISHED_PHASE:
            v_target = m_finalSpeed;
            break;
    }
    
    // 更新阶段时间
    m_phaseTime = (float)(m_phaseTime - last_m_phaseTime) / 1000.0f;
    last_m_phaseTime = m_phaseTime;
    
    return direction * v_target;
}

float SCurvePlanner::calculatePhaseTime(float startSpeed, float endSpeed, float acc) const {
    return (endSpeed - startSpeed) / acc;
}
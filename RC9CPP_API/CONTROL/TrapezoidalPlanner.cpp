#include "TrapezoidalPlanner.h"

TrapezoidalPlanner::TrapezoidalPlanner(pid &pidController)
    : V_start(0), V_max(0), V_end(0), R_acc(0), R_dec(0), S_total(0),
      pidControl(pidController), currentState(TARGET_REACHED) {}

void TrapezoidalPlanner::setTargetParameters(float vStart, float vMax, float vEnd, float accRatio, float decRatio, float totalDistance)
{
    V_start = vStart;
    V_max = vMax;
    V_end = vEnd;
    R_acc = accRatio;
    R_dec = decRatio;
    S_total = totalDistance;
    currentState = ACCELERATION; // 重置状态到加速阶段
}

// 更新状态机状态
void TrapezoidalPlanner::updateState(float position)
{
    float Sac = S_total * R_acc;
    float Sco = S_total * (1.0f - R_acc - R_dec);

    // 根据当前距离判断阶段
    if (position < Sac)
    {
        currentState = ACCELERATION;
    }
    else if (position < Sac + Sco)
    {
        currentState = CRUISE;
    }
    else if (position < S_total)
    {
        currentState = DECELERATION;
    }
    else
    {
        currentState = TARGET_REACHED;
    }
}

// 计算目标速度
float TrapezoidalPlanner::computeTargetVelocity(float position)
{
    // 更新当前状态
    updateState(position);

    float targetVelocity = 0.0f;
    float Sac = S_total * R_acc;
    float Sde = S_total * R_dec;
    float Aac = (V_max * V_max - V_start * V_start) / (2.0f * Sac);
    float Ade = (V_end * V_end - V_max * V_max) / (2.0f * Sde);

    switch (currentState)
    {
    case ACCELERATION:
        arm_sqrt_f32(V_start * V_start + 2.0f * Aac * position, &targetVelocity);
        break;
    case CRUISE:
        targetVelocity = V_max;
        break;
    case DECELERATION:
        arm_sqrt_f32(V_end * V_end + 2.0f * Ade * (S_total - position), &targetVelocity);
        break;
    case TARGET_REACHED:
        targetVelocity = 0.0f;
        break;
    }

    if (currentState == TARGET_REACHED && fabsf(position - S_total) < precisionThreshold)
    {
        currentState = PID_ADJUST;
    }

    // PID微调阶段
    if (currentState == PID_ADJUST)
    {
        float positionError = S_total - position;
        pidControl.setpoint = 0; // 目标误差为0
        targetVelocity = pidControl.PID_Compute(positionError);
    }

    return targetVelocity;
}

// 查询是否到达目标
bool TrapezoidalPlanner::isTargetReached() const
{
    return currentState == TARGET_REACHED;
}

// 获取当前状态名称
const char *TrapezoidalPlanner::getCurrentStateName() const
{
    switch (currentState)
    {
    case ACCELERATION:
        return "Acceleration";
    case CRUISE:
        return "Cruise";
    case DECELERATION:
        return "Deceleration";
    case PID_ADJUST:
        return "PID Adjustment";
    case TARGET_REACHED:
        return "Target Reached";
    default:
        return "Unknown";
    }
}

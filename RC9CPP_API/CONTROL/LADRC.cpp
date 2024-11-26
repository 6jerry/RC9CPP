#include "LADRC.h"

ladrc::ladrc(float r_, float w0_, float b0_, float kp_, float kd_, float output_limit_) : r(r_), w0(w0_), b0(b0_), output_limit(output_limit_), Kp(kp_), Kd(kd_)
{
    B1 = 3.0f * w0;
    B2 = 3.0f * w0 * w0;
    B3 = w0 * w0 * w0;
}

void ladrc::TD() // 传入原始期望 expect，输出v1,v2;v1为平滑后的期望值
{
    fh = -r * r * (V1 - expect) - 2.0f * r * V2;

    V1 += V2 * Ts;
    V2 += fh * Ts;
}

void ladrc::LESO() // 传入当前实际值，输出z1,z2,z3
{
    e = z1 - FeedBack;

    z1 += (z2 - B1 * e) * Ts; // 前状态估计
    z2 += (z3 - B2 * e + b0 * u) * Ts;
    z3 += -B3 * e * Ts; // 扰动补偿
}

void ladrc::LF()
{
    e1 = V1 - z1;
    e2 = V2 - z2;
    u0 = Kp * e1 + Kd * e2;
    if (b0 != 0)
    {
        u = u0 - (z3 / b0) * debug; // 有时候要u0和扰动补偿项单独调试
    }

    if (u > output_limit)
    {
        u = output_limit;
    }
    if (u < -output_limit)
    {
        u = -output_limit;
    }
}

float ladrc::ladrc_Compute(float Expect_, float FeedBack_)
{
    expect = Expect_;
    FeedBack = FeedBack_;

    uint32_t current_time = HAL_GetTick(); // 获取当前时间，单位ms
    if (previous_time != 0)
    { // 确保上一次时间不为0
        Ts = float(current_time - previous_time) / 1000.0f;
    }
    previous_time = current_time;

    TD();
    LESO();
    LF();

    return u;
}

void ladrc::ladrc_SetParameters(float r_, float w0_, float b0_, float kp_, float kd_, float debug_)
{
    r = r_;

    w0 = w0_;
    b0 = b0_;

    B1 = 3.0f * w0;
    B2 = 3.0f * w0 * w0;
    B3 = w0 * w0 * w0;

    Kd = kd_;
    Kp = kp_;
    debug = debug_;
}
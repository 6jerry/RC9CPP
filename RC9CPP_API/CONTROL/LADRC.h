#ifndef LADRC_H
#define LADRC_H

#ifdef __cplusplus
extern "C"
{
#endif
#include <stdbool.h>
#include "TaskManager.h"
#include "Serial_device.h"
#ifdef __cplusplus
}
#endif

#ifdef __cplusplus

class ladrc
{
private:
    /* data */
public:
    float Ts = 0.0f;                                                                                                                   // 采样周期
    float r = 0.0f, expect = 0.0f, V1 = 0.0f, V2 = 0.0f, fh = 0.0f;                                                                    // TD用的参数
    float z1 = 0.0f, z2 = 0.0f, z3 = 0.0f, y = 0.0f, b0 = 0.0f, e = 0.0f, B1 = 0.0f, B2 = 0.0f, B3 = 0.0f, FeedBack = 0.0f, w0 = 0.0f; // LESO用的参数
    float Kp = 0.0f, Kd = 0.0f, u = 0.0f, u0 = 0.0f, e1 = 0.0f, e2 = 0.0f;                                                             // LF用的参数

    float output_limit = 0.0f, debug = 1.0f;
    uint32_t previous_time = 0;
    ladrc(float r_, float w0_, float b0_, float kp_, float kd_, float output_limit_);
    void ladrc_SetParameters(float r_, float w0_, float b0_, float kp_, float kd_, float debug_);
    float ladrc_Compute(float Expect_, float FeedBack_);
    void TD();
    void LESO();
    void LF();
};

#endif
#endif
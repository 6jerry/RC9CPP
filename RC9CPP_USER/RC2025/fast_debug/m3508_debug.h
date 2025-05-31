#ifndef M3508_DEBUG_H
#define M3508_DEBUG_H

#ifdef __cplusplus
extern "C"
{
#endif
#include "debug_xbox.h"
#include "motor.h"
#include "RC9Protocol.h"
#ifdef __cplusplus
}
#endif
#ifdef __cplusplus

class m3508_m2006_debug :  public xbox_debug_base
{
private:
    float max_rpm = 0.0f, max_dis = 0.0f;

public:
    void not_start() override;
    power_motor *debug_motor = nullptr;
    RC9subscriber *msg_send = nullptr;

    void mode_2() override;
    void mode_3() override;
};

#endif
#endif
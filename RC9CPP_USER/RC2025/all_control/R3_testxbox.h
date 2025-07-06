#ifndef __R3_TEST_XBOX_H__
#define __R3_TEST_XBOX_H__

#ifdef __cplusplus
extern "C"
{
#endif
#include "debug_xbox.h"

#include "RC9Protocol.h"
#include "motor.h"

#ifdef __cplusplus
}
#endif
#ifdef __cplusplus

class R3_xbox : public xbox_debug_base
{
private:
public:
    power_motor *shoot_motor_1 = nullptr;
    power_motor *shoot_motor_2 = nullptr;
    float move_rpm = 3000.0f;
    void mode_1() override;
    void not_start() override;
    float target_rpm = 0.0f;
};

#endif
#endif

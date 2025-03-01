#ifndef RC_TEST_XBOX_H
#define RC_TEST_XBOX_H

#ifdef __cplusplus
extern "C"
{
#endif
#include "xbox.h"
#include "TaskManager.h"
#include "motor.h"
#ifdef __cplusplus
}
#endif
#ifdef __cplusplus

class yun_ball_xbox : public ITaskProcessor
{
private:
    power_motor *lfter_motor = nullptr, *turn_motor = nullptr;

public:
    void process_data();
    yun_ball_xbox();
    void btn_scan();
    void btnconfig_init();
    void add_motor(power_motor *lfter_motor_, power_motor *turn_motor_);
};

#endif
#endif
#ifndef SUPERYUNBALL_H
#define SUPERYUNBALL_H

#ifdef __cplusplus
extern "C"
{
#endif
#include "TaskManager.h"
#include "SuperPID.h"
#include "motor.h"
#include "gpio.h"
#include "CrsfReceiver.h"
#ifdef __cplusplus
}
#endif

#ifdef __cplusplus

class super_yunball : public ITaskProcessor
{
private:
    power_motor *yunball_motor1, *yunball_motor2;
    CrsfReceiver *remote_control = nullptr;

public:
    void
    process_data();
    void add_motors(power_motor *yunball_motor1_, power_motor *yunball_motor2_);
    void add_remote(CrsfReceiver *remote_control_);
    float max_rpm = 2000.0f;
};

#endif
#endif

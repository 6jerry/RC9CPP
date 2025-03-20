#ifndef AUTO_YUNBALL_H
#define AUTO_YUNBALL_H

#ifdef __cplusplus
extern "C"
{
#endif
#include "TaskManager.h"
#include "SuperPID.h"
#include "motor.h"
#include "PID.h"
#include "SuperPID.h"
#include "gpio.h"
#include "imu.h"

#include "xbox.h"
#include "debug_xbox.h"
#ifdef __cplusplus
}
#endif

#ifdef __cplusplus



class auto_yunball:public ITaskProcessor
{
};
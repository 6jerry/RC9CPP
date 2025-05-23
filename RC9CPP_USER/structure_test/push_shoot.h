#ifndef PUSH_SHOOT_H
#define PUSH_SHOOT_H

#ifdef __cplusplus
extern "C"
{
#endif

#include "FreeRTOS.h"
#include "task.h"
// #include <stdint.h>
#include <cmsis_os2.h>
#include "RC9Protocol.h"
#include "usart.h"
#include "TaskManager.h"
#include "debug_xbox.h"
#include "M3508.h"
#include "vesc.h"
#include "netswitch.h"
#include "rcncore.h"
#include "m6020.h"
#include "robot_chassis.h"
#include "tb6612.h"
#include "usb_device.h"
#include "odometry.h"
#include "servo.h"
#include "GCFSM.h"
#include "position.h"
    void pshoot_setup(void);
#ifdef __cplusplus
}
#endif
#ifdef __cplusplus
class demo : public ITaskProcessor, public chassis_user,public RC9subscriber
{
private:
    float elapsedTime = 0.0f, test_data = 0.0f, send_data = 6.0f;
    uint32_t currentTick = 0;
    uint32_t previousTick = 0;

    const float full_angle = 180.0f;

public:
    void process_data();
    float test_ccr = 0.0f, test2_ccr = 0.0f;
    uint32_t CCR = 0, CCR2 = 0;
};

#endif
#endif

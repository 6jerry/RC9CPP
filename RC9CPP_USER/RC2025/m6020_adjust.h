#ifndef M6020_ADJUST_H
#define M6020_ADJUST_H

#ifdef __cplusplus
extern "C"
{
#endif
#include <cmsis_os2.h>
#include "m6020.h"
#include "TaskManager.h"
#include "RC9Protocol.h"
#include "debug_xbox.h"
#include "M3508.h"
    void m6020_adjust_setup(void);
    void swerve_motor_adjust();
#ifdef __cplusplus
}
#endif
#ifdef __cplusplus

class demo : public ITaskProcessor, public chassis_user, public RC9subscriber
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

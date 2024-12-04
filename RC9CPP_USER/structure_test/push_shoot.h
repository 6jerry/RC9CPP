#ifndef PUSH_SHOOT_H
#define PUSH_SHOOT_H

#ifdef __cplusplus
extern "C"
{
#endif

#include "FreeRTOS.h"
#include "task.h"
// #include <stdint.h>
#include <cmsis_os.h>
#include "RC9Protocol.h"
#include "usart.h"
#include "TaskManager.h"
#include "shoot_xbox.h"
#include "M3508.h"
#include "vesc.h"
#include "netswitch.h"

    void pshoot_setup(void);
#ifdef __cplusplus
}
#endif
#ifdef __cplusplus
class demo : public ITaskProcessor, public rcnode
{
private:
    float testdd = 0.0f;

public:
    void process_data();
    uint8_t msgin(uint8_t rcnID_, const void *data) override;
    uint8_t msgout(uint8_t rcnID_, void *output) override;
    demo(float init_ = 0.0f);
    float testdata[6] = {0};
};

#endif
#endif

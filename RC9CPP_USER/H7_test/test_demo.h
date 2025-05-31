#ifndef TEST_DEMO_H
#define TEST_DEMO_H

#ifdef __cplusplus
extern "C"
{
#endif
#include "TaskManager.h"
#include "usart.h"
#include "RC9Protocol.h"
#include "fdcan_device.h"
#include "vesc.h"
#include "M3508.h"
#include "position.h"
    void test_demo(void);
#ifdef __cplusplus
}
#endif
#ifdef __cplusplus
class demo : public ITaskProcessor, public RC9subscriber
{
private:
    uint8_t ff = 0;
    /* data */
public:
    void process_data();

    float test12 = 0.0f;
    float delta_time = 0.0f;
    uint32_t previous_time = 0;
};

#endif
#endif
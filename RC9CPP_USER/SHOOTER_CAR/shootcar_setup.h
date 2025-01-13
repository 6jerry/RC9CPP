#ifndef SHOOTCAR_SETUP_H
#define SHOOTCAR_SETUP_H

#ifdef __cplusplus
extern "C"
{
#endif

#include "FreeRTOS.h"
#include "task.h"
// #include <stdint.h>
#include <cmsis_os.h>
#include "RC9Protocol.h"

#include "TaskManager.h"
#include "Action.h"
#include "M3508.h"
#include "chassis.h"
#include "netswitch.h"
#include "m6020.h"
#include "vesc.h"
    void shootcar_setup();

#ifdef __cplusplus
}
#endif
#ifdef __cplusplus
class demo : public ITaskProcessor
{
private:
    /* data */
public:
    void process_data();

    float test12 = 0.0f;
};

#endif
#endif
#ifndef R1N_SETUP_H
#define R1N_SETUP_H

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
#include "Action.h"
#include "M3508.h"
#include "chassis.h"
#include "r2n_xbox.h"
#include "m6020.h"
#include "go1can.h"

    void r1n_setup(void);
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
};

#endif
#endif

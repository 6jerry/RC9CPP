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

    void pshoot_setup(void);
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

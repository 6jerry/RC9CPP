#ifndef SIMPLE_TASK_H
#define SIMPLE_TASK_H

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
#include "xbox.h"
#include "m6020.h"

#include "Vector2D.h"

    void create_tasks(void);
#ifdef __cplusplus
}
#endif
#ifdef __cplusplus
#include "xbox.h"
// C 的接口函数，用于在 main.c 中调用
class demo : public ITaskProcessor
{
private:
    /* data */
public:
    void process_data();
};

#endif
#endif // SIMPLE_TASK_H

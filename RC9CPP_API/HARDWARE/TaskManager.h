
#ifndef TASK_MANAGER_H
#define TASK_MANAGER_H

#ifdef __cplusplus
extern "C"
{
#endif
#include <FreeRTOS.h>
#include <task.h>
// #include <stdint.h>
#include <cmsis_os.h>

#ifdef __cplusplus
}
#endif

#ifdef __cplusplus
// #include "RC9Protocol.h"
//  最大任务数和每个任务允许的类实例数
#define MAX_TASKS 10
#define MAX_CLASSES_PER_TASK 20
// 接口类

class ITaskProcessor
{
public:
    virtual void process_data() = 0; // 纯虚函数
    virtual ~ITaskProcessor() {};
    char name[16] = {'0'}; // 实例的名字，便于debug
};

// 任务信息结构体
struct TaskInfo
{
    TaskHandle_t handle = nullptr;
    char name[7];
    uint8_t Priority = 0; // 在1到5之间
    uint32_t delay_ms;
    ITaskProcessor *instances[MAX_CLASSES_PER_TASK] = {nullptr}; // 使用接口指针
    int instanceCount;
    uint32_t stack_size = 6 * 128;
};

// 任务管理者类
class TaskManager
{
public:
    TaskManager();
    void registerTask(int taskID, ITaskProcessor *instance); // 注册任务
                                                             // 存储任务信息

    void customize(int taskID, uint8_t priority, uint32_t delay_ms, uint32_t stack_size = 4 * 128); // 允许用户客制化指定的任务,但如果任务已经创建则客制化无效

private:
    static TaskInfo tasks[MAX_TASKS];
    static void TaskFunction(void *parameters); // 任务执行函数
};

#endif

#endif // TASK_MANAGER_H

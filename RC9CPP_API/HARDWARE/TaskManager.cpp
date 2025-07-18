#include "TaskManager.h"

TaskInfo TaskManager::tasks[MAX_TASKS];

TaskManager::TaskManager()
{
    for (int i = 0; i < MAX_TASKS; i++)
    {
        tasks[i].name[0] = 't';
        tasks[i].name[1] = 'a';
        tasks[i].name[2] = 's';
        tasks[i].name[3] = 'k';
        tasks[i].name[4] = '_';
        tasks[i].name[5] = '0' + i;
        tasks[i].name[6] = '\0';
        // 生成一波任务名字，便于后续debug
    }
    // 默认的任务参数设置，如有不满可以走客制化
    tasks[0].Priority = osPriorityRealtime;
    tasks[0].delay_ms = 1;
    tasks[1].Priority = osPriorityAboveNormal;
    tasks[1].delay_ms = 2;
    tasks[2].Priority = osPriorityAboveNormal;
    tasks[2].delay_ms = 5;
    tasks[3].Priority = osPriorityNormal;
    tasks[3].delay_ms = 5;
    tasks[4].Priority = osPriorityNormal;
    tasks[4].delay_ms = 10;
    tasks[5].Priority = osPriorityBelowNormal;
    tasks[5].delay_ms = 10;
    tasks[6].Priority = osPriorityBelowNormal;
    tasks[6].delay_ms = 15;
    tasks[7].Priority = osPriorityLow;
    tasks[7].delay_ms = 15;
    tasks[8].Priority = osPriorityLow;
    tasks[8].delay_ms = 20;
    tasks[9].Priority = osPriorityIdle;
    tasks[9].delay_ms = 40;
}

void TaskManager::registerTask(int taskID, ITaskProcessor *instance)
{
    if (taskID < 0 || taskID >= MAX_TASKS || tasks[taskID].instanceCount >= MAX_CLASSES_PER_TASK)
    {
        return;
    }
    // 注册
    tasks[taskID].instances[tasks[taskID].instanceCount] = instance;

    // 如果任务还未创建，则创建任务
    if (tasks[taskID].handle == nullptr)
    {
        osThreadAttr_t thread_attr = {};
        thread_attr.name = tasks[taskID].name;
        thread_attr.stack_size = tasks[taskID].stack_size;
        thread_attr.priority = tasks[taskID].Priority;

        tasks[taskID].handle = osThreadNew(TaskFunction, (void *)(intptr_t)taskID, &thread_attr);
    }
    tasks[taskID].instanceCount++;
}

void TaskManager::TaskFunction(void *parameters)
{
    int run_taskID = (int)(intptr_t)parameters;

    while (true)
    {
        for (int i = 0; i < tasks[run_taskID].instanceCount; ++i)
        { // 执行所有实例的任务处理需求
            if (tasks[run_taskID].instances[i])
            {
                tasks[run_taskID].instances[i]->process_data();
            }
        }
        osDelay(tasks[run_taskID].delay_ms); // 延时
    }
}

void TaskManager::customize(int taskID, osPriority_t priority, uint32_t delay_ms, uint32_t stack_size)
{
    if (tasks[taskID].handle != nullptr)
    {
        return; // 已创建的任务无法客制化
    }
    tasks[taskID].Priority = priority;
    tasks[taskID].delay_ms = delay_ms;
    tasks[taskID].stack_size = stack_size;
}
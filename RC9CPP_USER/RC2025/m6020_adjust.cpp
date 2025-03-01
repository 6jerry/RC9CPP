#include "m6020_adjust.h"

m6020s m6020_left_front(4, &hcan1), m6020_right_front(3, &hcan1), m6020_left_back(1, &hcan1), m6020_right_back(2, &hcan1); // 舵向电机
moters_debug_xbox m6020_debug;
TaskManager task_core;
CanManager can_core;
RC9Protocol esp_port(uart, &huart2);
demo test1;
extern "C"
{
    void m6020_adjust_setup(void)
    {
        can_core.init();
        esp_port.startUartReceiveIT();
        task_core.registerTask(0, &can_core);
        task_core.registerTask(3, &m6020_debug);
        task_core.registerTask(7, &test1);
        // task_core.registerTask(8, &esp_port);
        m6020_debug.add_motor(&m6020_left_front);
        m6020_debug.addport(&esp_port);
        osKernelStart();
    }
}

void demo::process_data()
{
}
#include "m6020_adjust.h"

m6020s m6020_front(3, &hcan1), m6020_left(1, &hcan1), m6020_right(2, &hcan1); // 舵向电机

moters_debug_xbox m6020_debug;
TaskManager task_core;
CanManager can_core;
RC9Protocol esp_port(uart, &huart2), debug_port(uart, &huart5);
demo test1;
extern "C"
{
    void m6020_adjust_setup(void)
    {
        can_core.init();
        esp_port.startUartReceiveIT();
        debug_port.initQueue();
        task_core.registerTask(0, &can_core);
        task_core.registerTask(3, &m6020_debug);
        task_core.registerTask(7, &debug_port);
        // task_core.registerTask(8, &esp_port);
        m6020_debug.add_motor(&m6020_right);
        m6020_debug.addport(&esp_port);
        m6020_right.rpm_pid.config_all(0.0f, 0.0f, 0.0f, 3000.0f, 2.0f, 65.0f);
        m6020_right.pos_pid.ConfigAll(0.0f, 0.0f, 0.0f, 0.0f, 130.0f, 0.2f, 0.0f);
        m6020_right.addport(&debug_port);
        m6020_right.start_debug();

        osKernelStart();
    }

    void swerve_motor_adjust()
    {
    }
}

void demo::process_data()
{
}
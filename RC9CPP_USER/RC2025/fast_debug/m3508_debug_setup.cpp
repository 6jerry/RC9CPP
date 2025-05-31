#include "m3508_debug_setup.h"
TaskManager task_core;
dji_motor_handle dji_core;
m3508p m2006(dji_id_1, &hfdcan1, 138.31579f, M2006_MAX_CURRENT, M2006_CURRENT_MAP);

RC9Protocol debug_port(uart, &huart4), esp_port(uart, &huart2);

m3508_m2006_debug debug_xbox;

extern "C"
{
    void m3508_debug_setup()
    {
        debug_port.startUartReceiveIT();
        debug_port.initQueue();
        esp_port.startUartReceiveIT();

        debug_xbox.addport(&esp_port);
        debug_xbox.msg_send->addport(&debug_port);
        debug_xbox.debug_motor = &m2006;

        m2006.rpm_control.config_all(0.0f, 0.0f, 0.0f, 0.0f, 5000.0f, 3.0f);
        m2006.angle_pid_control.ConfigAll(0.0f, 0.0f, 0.0f, 0.0f, 140.0f, 0.2f, 3.0f);

        task_core.registerTask(0, &dji_core);
        task_core.registerTask(8, &debug_xbox);
        task_core.registerTask(8, &debug_port);
        osKernelStart();
    }
}
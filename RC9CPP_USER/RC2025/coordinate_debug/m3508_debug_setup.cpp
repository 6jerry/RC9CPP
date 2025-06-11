#include "m3508_debug_setup.h"
TaskManager task_core;
dji_motor_handle dji_core;
m3508p m2006(dji_id_4, &hfdcan1, 111.72384f, M2006_MAX_CURRENT, M2006_CURRENT_MAP);

RC9Protocol debug_port(uart, &huart5), esp_port(uart, &huart3);

CrsfReceiver crsf_receiver(&huart7);

m3508_m2006_debug debug_xbox;

extern "C"
{
    void m3508_debug_setup()

    {
        CanDevice::InitAllFiltersNoMask();

        debug_port.initQueue();
        debug_port.startUartReceiveIT();
        esp_port.initQueue();

        esp_port.startUartReceiveIT();

        crsf_receiver.startUartReceiveIT();

        debug_xbox.addport(&esp_port);
        debug_xbox.msg_send->addport(&debug_port);
        debug_xbox.debug_motor = &m2006;

        m2006.rpm_control.config_all(12.0f, 0.9f, 8.6f, 0.0f, 10000.0f, 3.0f);
        m2006.angle_pid_control.ConfigAll(4.0f, 0.0f, 6.0f, 0.0f, 120.0f, 0.2f, 3.0f);

        task_core.registerTask(0, &dji_core);
        task_core.registerTask(8, &debug_xbox);
        task_core.registerTask(8, &debug_port);
        osKernelStart();
    }
}
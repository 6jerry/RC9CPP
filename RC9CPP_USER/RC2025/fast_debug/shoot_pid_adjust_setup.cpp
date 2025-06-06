#include "shoot_pid_adjust_setup.h"
TaskManager task_core;

vesc shoot_m(vesc_id_4, &hfdcan2, 7.0f, 2.0f);
RC9Protocol debug_port(uart, &huart5), esp_port(uart, &huart3);
Encoder can_encoder(1, &hfdcan3);
shoot_pid_xbox ad_xbox;
extern "C"
{

    void shoot_pid_adjust_setup(void)
    {
        CanDevice::InitAllFiltersNoMask();

        debug_port.initQueue();
        debug_port.startUartReceiveIT();
        esp_port.initQueue();

        esp_port.startUartReceiveIT();

        ad_xbox.addport(&esp_port);
        ad_xbox.msg_send->addport(&debug_port);
        ad_xbox.shoot_motor = &shoot_m;
        ad_xbox.encoder = &can_encoder;

        ad_xbox.shoot_control.ConfigAll(10000.0f, 3.3f, 64.0f, 0.0f, 1800.0f, 0.001f, 0.02f);
        task_core.registerTask(1, &shoot_m);
        task_core.registerTask(2, &ad_xbox);
        task_core.registerTask(8, &debug_port);
        osKernelStart();
    }
}

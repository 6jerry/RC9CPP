#include "R3_test.h"

TaskManager task_core;
dji_motor_handle dji_core;

RC9Protocol esp_port(uart, &huart3), debug_port(uart, &huart4);

vesc shoot_1(vesc_id_1, &hfdcan1, 7.0f, 1.0f);
vesc shoot_2(vesc_id_2, &hfdcan1, 7.0f, 1.0f);

R3_xbox test_xbox;
demo plot;

extern "C"
{
    void r3_setup()
    {
        CanDevice::InitAllFiltersNoMask();
        esp_port.initQueue();
        esp_port.startUartReceiveIT();
        debug_port.startUartReceiveIT();
        debug_port.initQueue();

        test_xbox.addport(&esp_port);

        shoot_1.rpm_control.config_all(176.0f, 1.0f, 86.0f, 20.0f, 70000, 5.0f);
        shoot_2.rpm_control.config_all(140.0f, 0.48f, 0.86f, 20.0f, 70000, 5.0f);
        shoot_1.rpm_control.enable_TD();
        shoot_2.rpm_control.enable_TD();
        // shoot_1.addport(&debug_port);
        // shoot_1.start_debug();

        plot.addport(&debug_port);

        test_xbox.shoot_motor_1 = &shoot_1;
        test_xbox.shoot_motor_2 = &shoot_2;
        task_core.registerTask(0, &dji_core);
        task_core.registerTask(0, &shoot_1);
        task_core.registerTask(0, &shoot_2);
        task_core.registerTask(4, &test_xbox);
        task_core.registerTask(8, &debug_port);
        task_core.registerTask(7, &plot);
        osKernelStart();
    }
}

void demo::process_data()
{
    float send_datas[3] = {shoot_1.get_rpm(),
                           shoot_2.get_rpm(),
                           test_xbox.target_rpm};

    sendFloatData(1, send_datas, 3);
}
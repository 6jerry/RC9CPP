#include "Up_setup.h"

TaskManager task_core;
CanManager can_core;
RC9Protocol esp_port(uart, &huart2), debug_port(uart, &huart5);
m3508p lifter(1, &hcan1), turnner(2, &hcan1), pitcher(3, &hcan1);
vesc m8080(4, &hcan2, 7.0f, 1.0f);

UserCtrl_xbox xbox_test;

extern "C"
{
    void mytest(void)
    {
        can_core.init();

        esp_port.startUartReceiveIT();
        debug_port.initQueue();

        // 俯仰电机
        pitcher.start_debug();
        pitcher.addport(&debug_port);
        pitcher.config_mech_param(19.2032f, 35.0f);
        pitcher.distance_pid_control.ConfigAll(4.0f, 0.0f, 0.086f, 0.0f, 430.0f, 1.0f, 0.0f);

        xbox_test.addport(&esp_port);
        xbox_test.add_motor(&pitcher, &lifter, &m8080, &turnner);
        xbox_test.add_trigger(GPIOC, GPIO_PIN_15, GPIOC, GPIO_PIN_13); // shooter c 13

        task_core.registerTask(0, &can_core);
        task_core.registerTask(3, &xbox_test);
        task_core.registerTask(8, &debug_port);

        osKernelStart();
    }
}

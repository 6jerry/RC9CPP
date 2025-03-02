#include "YunBall_test.h"

TaskManager task_core;
CanManager can_core;
RC9Protocol esp_port(uart, &huart2);

m3508p shooter(3, &hcan2), pitcher(1, &hcan2), lifter(2, &hcan2), turnner(4, &hcan2); // 抬升电机，俯仰电机

yun_ball_xbox xbox_test;
extern "C"
{
    void yunball_test_setup(void)
    {
        can_core.init();
        esp_port.startUartReceiveIT();
        xbox_test.addport(&esp_port);
        xbox_test.add_motor(&pitcher, &lifter);
        xbox_test.add_trigger(GPIOC, GPIO_PIN_15);
        task_core.registerTask(0, &can_core);
        task_core.registerTask(3, &xbox_test);
        osKernelStart();
    }
}

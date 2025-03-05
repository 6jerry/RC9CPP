#include "YunBall_test.h"

TaskManager task_core;
CanManager can_core;
RC9Protocol esp_port(uart, &huart2);

m3508p shooter(3, &hcan2), pitcher(1, &hcan2), lifter(2, &hcan2), turnner(4, &hcan2); // 抬升电机，俯仰电机

m6020s m6020_front(3, &hcan1), m6020_left(1, &hcan1), m6020_right(2, &hcan1); // 舵向电机
vesc vesc1(1, &hcan1), vesc2(2, &hcan1), vesc3(3, &hcan1);
yun_ball_xbox xbox_test;
extern "C"
{
    void yunball_test_setup(void)
    {
        can_core.init();
        esp_port.startUartReceiveIT();
        xbox_test.addport(&esp_port);
        xbox_test.add_motor(&pitcher, &lifter, &shooter, &turnner);
        xbox_test.add_trigger(GPIOC, GPIO_PIN_15, GPIOC, GPIO_PIN_13); // shooter c 13
        task_core.registerTask(0, &can_core);
        task_core.registerTask(3, &xbox_test);
        task_core.registerTask(2, &vesc1);
        task_core.registerTask(2, &vesc2);
        task_core.registerTask(2, &vesc3);

        osKernelStart();
    }
}

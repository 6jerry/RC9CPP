#include "r2n_setup.h"
RC9Protocol esp32_serial(&huart1, false),
    data_chain(&huart5, true);
m3508p m3508_front(1, &hcan1), m3508_left(3, &hcan1), m3508_right(2, &hcan1); // 九期r2，硬件连接：三只m3508作为底盘动力电机位于can1
TaskManager task_core;
CanManager can_core;
action Action(&huart3, -160.0f, 120.0f, true);
omni3 r2n_chassis(&m3508_front, &m3508_right, &m3508_left, 0.0719f, 0.406f, &Action, 7.0f, 0.0f, 0.7f, 0.0086f, 0.0f, 0.026f);
xbox_r2n r2_remote(&Action, &r2n_chassis);

extern "C" void r2n_setup(void)
{
    can_core.init();
    Action.startUartReceiveIT();
    esp32_serial.startUartReceiveIT();

    data_chain.startUartReceiveIT();
    esp32_serial.addsubscriber(&r2_remote);

    task_core.registerTask(0, &can_core);
    task_core.registerTask(3, &r2n_chassis);
    task_core.registerTask(2, &r2_remote);
    osKernelStart();
}
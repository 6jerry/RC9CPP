#include "superyunball_setup.h"
TaskManager task_core;

vesc shoot_1(vesc_id_1, &hfdcan1, 7.0f, 1.0f);
vesc shoot_2(vesc_id_2, &hfdcan1, 7.0f, 1.0f);
RC9Protocol position_port(uart, &huart6);
CrsfReceiver remote_controller(&huart2);

super_yunball yun_ball_core;
extern "C"
{
    void yunball_setup()
    {
        CanDevice::InitAllFiltersNoMask();
        position_port.initQueue();
        position_port.startUartReceiveIT();
        remote_controller.startUartReceiveIT();
        yun_ball_core.add_motors(&shoot_1, &shoot_2);
        yun_ball_core.add_remote(&remote_controller);

        task_core.registerTask(2, &shoot_1);
        task_core.registerTask(3, &shoot_2);
        task_core.registerTask(4, &yun_ball_core);
        osKernelStart();
    }
}
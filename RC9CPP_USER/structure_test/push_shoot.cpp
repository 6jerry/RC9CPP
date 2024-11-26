#include "push_shoot.h"

m3508p m3508_shooter(1, &hcan1), m3508_pitch(2, &hcan1);

TaskManager task_core;
CanManager can_core;
shoot_xbox shoot_control(&m3508_shooter, &m3508_pitch);
RC9Protocol debug(&huart5, false), esp32_serial(&huart1, false);

demo test2;
extern "C" void pshoot_setup(void)
{
    can_core.init();

    debug.startUartReceiveIT();
    esp32_serial.startUartReceiveIT();
    esp32_serial.addsubscriber(&shoot_control);
    task_core.registerTask(0, &can_core);
    task_core.registerTask(1, &shoot_control);
    task_core.registerTask(8, &debug);
    task_core.registerTask(8, &test2);

    osKernelStart();
}

void demo::process_data()
{
}
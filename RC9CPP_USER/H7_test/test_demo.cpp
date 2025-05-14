#include "test_demo.h"
TaskManager task_core;
RC9Protocol usb_port(uart, &huart10);
demo test2, test3;

m3508p m3508_test(dji_id_6, &hfdcan1), m35082(dji_id_6, &hfdcan2);
dji_motor_handle dji_core;

extern "C" void
test_demo(void)
{
    CanDevice::InitAllFiltersNoMask();
    usb_port.initQueue();
    usb_port.startUartReceiveIT();
    task_core.registerTask(0, &dji_core);
    task_core.registerTask(8, &test3);
    task_core.registerTask(7, &usb_port);
    test3.addport(&usb_port);
    // 在这里编写测试代码
    osKernelStart();
}

void demo::process_data()
{
    // 在这里编写测试代码
    uint32_t current_time = HAL_GetTick();
    delta_time = float(current_time - previous_time) / 1000.0f;
    previous_time = current_time;

    float testd[3] = {0.3f, 0.2f, 0.1f};
    sendFloatData(1, testd, 3);
}
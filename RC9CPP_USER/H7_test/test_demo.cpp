#include "test_demo.h"
TaskManager task_core;
RC9Protocol usb_port(uart, &huart3);
demo test2, test3;

m3508p m3508_test(dji_id_1, &hfdcan1, true, 138.31579f, M2006_MAX_CURRENT, M2006_CURRENT_MAP);

vesc u8_test(vesc_id_1, &hfdcan3);

dji_motor_handle dji_core;
position position_imu;

//lg dl 3.10344
//3.84211

extern "C" void
test_demo(void)
{
    CanDevice::InitAllFiltersNoMask();
    usb_port.initQueue();
    usb_port.startUartReceiveIT();
    position_imu.addport(&usb_port);
  task_core.registerTask(0, &dji_core);
    task_core.registerTask(5, &test3);
    task_core.registerTask(5, &usb_port);
    task_core.registerTask(1, &u8_test);
    // test3.addport(&usb_port);
    //  在这里编写测试代码

    osKernelStart();
}

void demo::process_data()
{
    // 在这里编写测试代码
    //    uint32_t current_time = HAL_GetTick();
    //    delta_time = float(current_time - previous_time) / 1000.0f;
    //    previous_time = current_time;
    //    test12 += 0.001f;
    //    float testd[3] = {test12, 0.2f, 0.1f};
    //    sendFloatData(1, testd, 3);
    //    u8_test.send_rpm(80.0f);
    if (ff == 0)
    {
        position_imu.imu_rst();
        ff = 1;
    }
}
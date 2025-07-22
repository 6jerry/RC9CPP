#include "position_setup.h"
TaskManager task_core;
RC9Protocol to_stm32(&huart3, false);

Oencoder encoder_test(&htim1, &htim3);
HWT101CT imu_101(&huart6);

extern "C" void position_setup()
{
    imu_101.startUartReceiveIT();
    to_stm32.startUartReceiveIT(); 
    imu_101.add_io(GPIOC, GPIO_PIN_8);
    task_core.registerTask(1, &encoder_test);
    task_core.registerTask(3, &to_stm32);
    task_core.registerTask(8, &imu_101);

    encoder_test.init();
    encoder_test.add_imu(&imu_101);
    encoder_test.addport(&to_stm32);

    osKernelStart();
}

void demo::process_data()
{
    currentTick = HAL_GetTick(); // ???? tick ?

    // ??????????????
    elapsedTime = (float)(currentTick - previousTick);

    if (elapsedTime >= 1) // ??????? 1 ??
    {
        previousTick = currentTick; // ????? tick ?
                                    //  ????????????
                                    //  ??,LED???
    }

    sendFloatData(1, &test_data, 1);
}
void demo::DataReceivedCallback(const uint8_t *byteData, const float *floatData, uint8_t id, uint16_t byteCount)
{
    test_data = floatData[0];
}

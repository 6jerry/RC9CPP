#include "rescar_setup.h"

TaskManager task_core;
RC9Protocol debug(&huart5, false), esp32_serial(&huart2, false);

fdi fdi_test(&huart4);
tb6612 right_front(&htim2, TIM_CHANNEL_1, &htim8, GPIOC, GPIO_PIN_0, GPIOC, GPIO_PIN_1), right_back(&htim2, TIM_CHANNEL_2, &htim4, GPIOC, GPIO_PIN_2, GPIOC, GPIO_PIN_3), left_front(&htim2, TIM_CHANNEL_3, &htim3, GPIOF, GPIO_PIN_1, GPIOF, GPIO_PIN_2), left_back(&htim2, TIM_CHANNEL_4, &htim1, GPIOF, GPIO_PIN_3, GPIOF, GPIO_PIN_4);

shit shit1;
extern "C" void rescar_setup(void)
{

    right_front.init();
    right_back.init();
    left_front.init();
    left_back.init();

    debug.startUartReceiveIT();
    esp32_serial.startUartReceiveIT();
    fdi_test.startUartReceiveIT();

    task_core.registerTask(4, &right_front);
    task_core.registerTask(4, &right_back);
    task_core.registerTask(5, &left_front);
    task_core.registerTask(5, &left_back);
    task_core.registerTask(8, &debug);
    task_core.registerTask(7, &shit1);

    osKernelStart();
}

void shit::process_data()
{
}
#include "yunball_test_setup.h"

TaskManager task_core;
dji_motor_handle dji_core;
m3508p turn_motor(dji_id_2, &hfdcan2, 49.1372f);
yunball_test_xbox xbox;
RC9Protocol esp_port(uart, &huart3);

extern "C" {
    void yunball_test_setup()
    {
        CanDevice::InitAllFiltersNoMask();
        esp_port.initQueue();
        esp_port.startUartReceiveIT();

        xbox.addport(&esp_port);
        xbox.add_motor(&turn_motor);
        xbox.add_io(GPIOG, GPIO_PIN_6, GPIOG, GPIO_PIN_8, GPIOG, GPIO_PIN_5);   //7发射， 8夹爪，6抬升， 5推射

        task_core.registerTask(0, &dji_core);
        task_core.registerTask(2, &xbox);

        osKernelStart();
    }
}
#include "yunball_test_setup.h"

TaskManager task_core;
dji_motor_handle dji_core;
m3508p turn_motor(dji_id_2, &hfdcan1, 49.1372f);
yunball_test_xbox xbox;
RC9Protocol esp_port(uart, &huart3);
auto_yunball yunball_port;

extern "C" {
    void yunball_test_setup()
    {
        CanDevice::InitAllFiltersNoMask();
        esp_port.initQueue();
        esp_port.startUartReceiveIT();

        xbox.addport(&esp_port);
				yunball_port.add_motor(&turn_motor);
				yunball_port.add_io(GPIOG, GPIO_PIN_6, GPIOG, GPIO_PIN_8, GPIOG, GPIO_PIN_5);   //7发射， 8夹爪，6抬升， 5推射
				xbox.add_autoyunball(&yunball_port);
        //xbox.add_motor(&turn_motor);
        //xbox.add_io(GPIOG, GPIO_PIN_6, GPIOG, GPIO_PIN_8, GPIOG, GPIO_PIN_5);   //7发射， 8夹爪，6抬升， 5推射
        turn_motor.rpm_control.config_all(32.0f, 0.76f, 8.6f, 106.0f, 20000.0f, 5.0f);

        task_core.registerTask(0, &dji_core);
        task_core.registerTask(2, &xbox);
				task_core.registerTask(5, &yunball_port);
        osKernelStart();
    }
}
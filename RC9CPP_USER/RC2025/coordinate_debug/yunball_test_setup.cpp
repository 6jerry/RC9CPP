#include "yunball_test_setup.h"

TaskManager task_core;
dji_motor_handle dji_core;
m3508p turn_motor(dji_id_2, &hfdcan1, 49.1372f);
yunball_test_xbox xbox;
RC9Protocol esp_port(uart, &huart3);
RC9Protocol send_port(uart, &huart5);
auto_yunball yunball_port;

Encoder encoder(0x01, &hfdcan3);
PolynomialFitter fitter(0.05f, 0.23f, false);
vesc m6374(vesc_id_4, &hfdcan2, 7.0f, 2.0f);
AutoShooter auto_shooter;

extern "C" {
    void yunball_test_setup()
    {
        CanDevice::InitAllFiltersNoMask();
        esp_port.initQueue();
        esp_port.startUartReceiveIT();
        send_port.initQueue();
        send_port.startUartReceiveIT();

        xbox.addport(&esp_port);
				yunball_port.add_motor(&turn_motor);
        yunball_port.add_shooter(&auto_shooter);
				yunball_port.add_io(GPIOG, GPIO_PIN_6, GPIOG, GPIO_PIN_8, GPIOG, GPIO_PIN_5, GPIOD, GPIO_PIN_14);   //7发射， 8夹爪，6抬升， 5推射
				xbox.add_autoyunball(&yunball_port);
        xbox.add_motor(&turn_motor);
        xbox.add_io(GPIOD, GPIO_PIN_14); 
        turn_motor.rpm_control.config_all(32.0f, 0.76f, 8.6f, 106.0f, 20000.0f, 5.0f);

        // auto_shooter
        auto_shooter.add_encoder(&encoder);
        auto_shooter.add_motor(&m6374);
        auto_shooter.dis_control.ConfigAll(16000.0f, 3.3f, 64.0f, 0.0f, 1800.0f, 0.001f, 0.015f);
        auto_shooter.add_plan_info(400, 400, 1200, 400, 400);
        auto_shooter.add_trigger(GPIOF, GPIO_PIN_5, GPIOG, GPIO_PIN_7);
        auto_shooter.add_fitter(&fitter);

        task_core.registerTask(0, &dji_core);
        task_core.registerTask(2, &xbox);
				task_core.registerTask(5, &yunball_port);
        task_core.registerTask(2, &auto_shooter);
        task_core.registerTask(1, &m6374);
        osKernelStart();
    }
}
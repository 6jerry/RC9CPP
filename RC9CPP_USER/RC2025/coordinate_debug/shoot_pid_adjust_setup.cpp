#include "shoot_pid_adjust_setup.h"
TaskManager task_core;

vesc shoot_m(vesc_id_4, &hfdcan2, 7.0f, 2.0f);
vesc u8_front(vesc_id_1, &hfdcan2), u8_left(vesc_id_2, &hfdcan2), u8_right(vesc_id_3, &hfdcan2);
RC9Protocol debug_port(uart, &huart5), esp_port(uart, &huart3);
Encoder can_encoder(1, &hfdcan3);
shoot_pid_xbox ad_xbox;
PolynomialFitter fitter(0.05f, 0.23f, false);
AutoShooter shoot_core;

CrsfReceiver crsf_receiver(&huart7);

extern "C"
{

    void shoot_pid_adjust_setup(void)
    {
        CanDevice::InitAllFiltersNoMask();

        debug_port.initQueue();
        debug_port.startUartReceiveIT();
        esp_port.initQueue();

        esp_port.startUartReceiveIT();

        crsf_receiver.startUartReceiveIT();

        shoot_core.add_encoder(&can_encoder);
        shoot_core.add_motor(&shoot_m);
        shoot_core.dis_control.ConfigAll(12000.0f, 3.3f, 128.0f, 0.0f, 1800.0f, 0.001f, 0.015f);
        shoot_core.add_trigger(GPIOF, GPIO_PIN_5, GPIOG, GPIO_PIN_7);
        shoot_core.add_fitter(&fitter);

        ad_xbox.addport(&esp_port);
        ad_xbox.msg_send->addport(&debug_port);
        ad_xbox.shooter = &shoot_core;

        task_core.registerTask(1, &shoot_m);
        task_core.registerTask(3, &ad_xbox);
        task_core.registerTask(2, &shoot_core);
        task_core.registerTask(8, &debug_port);
        task_core.registerTask(1, &u8_front);
        task_core.registerTask(1, &u8_left);
        task_core.registerTask(1, &u8_right);
        osKernelStart();
    }
}

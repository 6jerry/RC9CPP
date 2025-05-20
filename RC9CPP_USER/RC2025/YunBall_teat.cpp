#include "YunBall_test.h"

TaskManager task_core;
CanManager can_core;
RC9Protocol esp_port(uart, &huart2), debug_port(uart, &huart5);

AutoShooter autoshooter;
// 编码器
Encoder encoder(&huart6);
// 维特IMU
wit_gyro wit_imu(&huart4);

serialStudio test_port;
m3508p lifter(1, &hcan1, true), turnner(2, &hcan1, true, 49.1372f), pithcer(3, &hcan1);

vesc m8080(4, &hcan2, 7.0f, 1.0f);
yun_ball_xbox xbox_test;
auto_yunball yunball_core;

extern "C"
{

    void yunball_test_setup(void)
    {
        encoder.startUartReceiveIT();
        wit_imu.startUartReceiveIT();
        esp_port.startUartReceiveIT();

        debug_port.initQueue();
        can_core.init();
        autoshooter.add_imu(&encoder, &wit_imu);
        // PD7 夹爪 PG10 射球 PG11气阀 PE2 棘轮
        autoshooter.add_trigger(GPIOF, GPIO_PIN_5, GPIOB, GPIO_PIN_4, GPIOA, GPIO_PIN_7);
        autoshooter.add_motor(&m8080, &pithcer);
        autoshooter.add_plan_info(400.0f, 400.0f, 1200.0f, 200.0f, 200.0f);

        xbox_test.add_auto_shooter(&autoshooter);
        xbox_test.addport(&esp_port);
        xbox_test.add_motor(&lifter, &turnner);
        xbox_test.add_trigger(GPIO_PIN_7, GPIOD);

        debug_port.startUartReceiveIT();

        task_core.registerTask(0, &can_core);
        task_core.registerTask(1, &xbox_test);
        task_core.registerTask(2, &autoshooter);
        task_core.registerTask(3, &m8080);
        task_core.registerTask(7, &debug_port);
        m8080.rpm_control.config_all(230.0f, 2.2f, 486.0f, 0.0f, 50000.0f, 6.0f);

        osKernelStart();
    }

    void auto_shooter_setup(void)
    {

        debug_port.startUartReceiveIT();
        encoder.startUartReceiveIT();
        wit_imu.startUartReceiveIT();
        esp_port.startUartReceiveIT();
        test_port.addport(&debug_port);
        can_core.init();
        lifter.config_mech_param(19.2032f, 35.0f);
        autoshooter.add_imu(&encoder, &wit_imu);
        // PD7 夹爪 PG10 射球 PG11气阀 PE2 棘轮
        autoshooter.add_trigger(GPIOF, GPIO_PIN_5, GPIOE, GPIO_PIN_2, GPIOG, GPIO_PIN_10);
        autoshooter.add_motor(&m8080, &pithcer);
        autoshooter.add_plan_info(600.0f, 400.0f, 1400.0f, 600.0f, 200.0f);

        xbox_test.add_auto_shooter(&autoshooter);
        xbox_test.addport(&esp_port);
        xbox_test.add_motor(&lifter, &turnner);
        xbox_test.add_trigger(GPIO_PIN_7, GPIOD);
        xbox_test.add_yunball(&yunball_core);
        xbox_test.add_serial_studio(&test_port);
        yunball_core.add_io(GPIOA, GPIO_PIN_7, GPIOF, GPIO_PIN_6, GPIOD, GPIO_PIN_7, GPIOG, GPIO_PIN_11);
        yunball_core.add_motor(&lifter, &turnner);

        task_core.registerTask(0, &can_core);
        task_core.registerTask(3, &xbox_test);
        task_core.registerTask(4, &autoshooter);
        task_core.registerTask(5, &m8080);
        task_core.registerTask(6, &yunball_core);
        task_core.registerTask(9, &debug_port);
        m8080.rpm_control.config_all(230.0f, 2.2f, 486.0f, 0.0f, 50000.0f, 6.0f);

        osKernelStart();
    }
}

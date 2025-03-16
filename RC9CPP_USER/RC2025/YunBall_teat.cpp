#include "YunBall_test.h"

TaskManager task_core;
CanManager can_core;
RC9Protocol esp_port(uart, &huart2), debug_port(uart, &huart5);

LaserProcessor laser(&huart6);

m3508p shooter(2, &hcan1), m3508_left(4, &hcan1, true), m3508_front(3, &hcan1, true), m3508_right(1, &hcan1, true); // 抬升电机，俯仰电机

moters_debug_xbox m3508_debuger;

debug_xbox shooter_debuger;

BallShooter shooter_core;

chassis_info s3_chassis_info = {0.37f, 0.17f, 0.3f, 0.0f, 0.44f, 0.38735f};
// m6020s m6020_front(3, &hcan1), m6020_left(1, &hcan1), m6020_right(2, &hcan1); // 舵向电机
vesc vesc_front(1, &hcan2, 21, 3.0f),
    vesc_left(2, &hcan2, 21, 3.0f), vesc_right(3, &hcan2, 21, 3.0f), m8080(4, &hcan2, 7.0f, 1.0f);
yun_ball_xbox xbox_test;

RoboChassis s3_chassis(swerve3_chassis);

chassis_debug_xbox s3_xbox(20.0f, 12.0f);
extern "C"
{
    void yunball_test_setup(void)
    {

        laser.SendInitCommands();
        laser.startUartReceiveIT();
        esp_port.startUartReceiveIT();
        can_core.init();
        debug_port.initQueue();
        m3508_left.start_debug();
        m3508_left.addport(&debug_port);
        m3508_left.config_mech_param(19.2032f, 35.0f); // 光电门f6
        xbox_test.addport(&esp_port);
        xbox_test.add_motor(&shooter, &m3508_front, &m8080, &m3508_right);
        xbox_test.add_trigger(GPIOC, GPIO_PIN_15, GPIOC, GPIO_PIN_13); // shooter c 13
        task_core.registerTask(0, &can_core);
        task_core.registerTask(3, &xbox_test);
        task_core.registerTask(2, &m8080);
        // task_core.registerTask(2, &vesc2);
        // task_core.registerTask(2, &vesc3);
        task_core.registerTask(8, &debug_port);

        osKernelStart();
    }

    void m3508_adjust(void)
    {
        can_core.init();
        esp_port.startUartReceiveIT();
        debug_port.initQueue();
        m3508_front.start_debug();
        m3508_front.addport(&debug_port);

        m3508_left.distance_pid_control.ConfigAll(4.0f, 0.0f, 0.086f, 0.0f, 430.0f, 1.0f, 0.0f);

        m3508_debuger.addport(&esp_port);
        m3508_debuger.add_motor(&m3508_front);
        m3508_left.config_mech_param(19.2032f, 35.0f);

        m3508_front.config_mech_param(48.26f, 0.0f);
        m3508_front.angle_pid_control.ConfigAll(3.1f, 0.4f, 1.4f, 0.0f, 160.0f, 0.2f, 3.0f);
        task_core.registerTask(0, &can_core);
        task_core.registerTask(3, &m3508_debuger);
        task_core.registerTask(8, &debug_port);

        osKernelStart();
    }

    void shooter_adjust(void)
    {
        laser.SendInitCommands();
        laser.startUartReceiveIT();
        esp_port.startUartReceiveIT();
        can_core.init();
        debug_port.initQueue();

        shooter_debuger.addport(&esp_port);
        shooter_core.add_moter(&m8080);
        shooter_core.addxbox(&shooter_debuger);
        shooter_core.addport(&debug_port);
        shooter_core.add_laser(&laser);
        task_core.registerTask(1, &m8080);
        task_core.registerTask(0, &can_core);
        task_core.registerTask(8, &debug_port);
        task_core.registerTask(3, &shooter_debuger);
        task_core.registerTask(7, &shooter_core);
        shooter_core.pull_dis_control.config_all(0.0f, 0.0f, 0.0f, 0.0f, 50000.0f, 0.003f);

        m8080.rpm_control.config_all(200.0f, 0.48f, 86.0f, 0.0f, 50000.0f, 6.0f);

        osKernelStart();
    }

    void chassis_adjust(void)
    {
        can_core.init();
        esp_port.startUartReceiveIT();
        debug_port.initQueue();

        m3508_front.config_mech_param(48.26f, 0.0f);
        m3508_front.angle_pid_control.ConfigAll(3.1f, 0.4f, 1.4f, 0.0f, 160.0f, 0.2f, 3.0f);

        m3508_left.config_mech_param(48.26f, 0.0f);
        m3508_left.angle_pid_control.ConfigAll(3.1f, 0.4f, 1.4f, 0.0f, 160.0f, 0.2f, 3.0f);

        m3508_right.config_mech_param(48.26f, 0.0f);
        m3508_right.angle_pid_control.ConfigAll(3.1f, 0.4f, 1.4f, 0.0f, 160.0f, 0.2f, 3.0f);

        s3_chassis.config(s3_chassis_info);
        s3_chassis.add_6_motors(&m3508_front, &vesc_front, &m3508_right, &vesc_right, &m3508_left, &vesc_left);

        s3_chassis.add_photogate(GPIOF, GPIO_PIN_11, GPIOF, GPIO_PIN_12, GPIOF, GPIO_PIN_13, GPIOF, GPIO_PIN_10);

        s3_xbox.addport(&esp_port);
        s3_xbox.add_chassis(&s3_chassis);
        task_core.registerTask(1, &vesc_front);
        task_core.registerTask(1, &vesc_left);
        task_core.registerTask(1, &vesc_right);

        task_core.registerTask(0, &can_core);
        task_core.registerTask(2, &s3_chassis);
        task_core.registerTask(3, &s3_xbox);
        osKernelStart();
    }
}

#include "chassis_test.h"

TaskManager task_core;
CanManager can_core;
RC9Protocol esp_port(uart, &huart2), debug_port(uart, &huart5);

m3508p shooter(2, &hcan1), m3508_left(4, &hcan1, true), m3508_front(3, &hcan1, true), m3508_right(1, &hcan1, true);

chassis_info s3_chassis_info = {0.037f, 0.17f, 0.3f, 0.0f, 0.44f, 0.38735f};

vesc vesc_front(1, &hcan2, 21.0f, 3.0f),
    vesc_left(2, &hcan2, 21.0f, 3.0f), vesc_right(3, &hcan2, 21.0f, 3.0f);

RoboChassis s3_chassis(swerve3_chassis);

chassis_debug_xbox s3_xbox(8.0f, 6.0f);

extern "C"
{
    void chassis_move_test(void)
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

        s3_xbox.init_plan(0.09f, 0.01f);

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
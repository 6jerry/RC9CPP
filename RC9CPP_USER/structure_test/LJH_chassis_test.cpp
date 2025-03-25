#include "LJH_chassis_test.h"

TaskManager task_core;
CanManager can;
RC9Protocol esp(uart, &huart2), debug(uart, &huart5);

m3508p m3508_left(4, &hcan1, true), m3508_front(3, &hcan1, true), m3508_right(1, &hcan1, true);

chassis_info chassis_info_3 = {0.037f, 0.17f, 0.3f, 0.0f, 0.44f, 0.38735f};
vesc vesc_front(1, &hcan2, 21.0f, 3.0f), vesc_left(2, &hcan2, 21.0f, 3.0f), vesc_right(3, &hcan2, 21.0f, 3.0f);
RoboChassis chassis_3(swerve3_chassis);

chassis_debug_xbox xbox3(8.0f, 6.0f); //最高速度为8.0m/s, 最高角速度为6.0rad/s

extern "C"
{
    void chassis_test(void)
    {
        can.init();
        esp.startUartReceiveIT();
        debug.initQueue();

        m3508_front.config_mech_param(48.26f, 0.0f);
        m3508_front.angle_pid_control.ConfigAll(3.1f, 0.4f, 1.4f, 0.0f, 160.0f, 0.2f, 3.0f);

        m3508_left.config_mech_param(48.26f, 0.0f);
        m3508_left.angle_pid_control.ConfigAll(3.1f, 0.4f, 1.4f, 0.0f, 160.0f, 0.2f, 3.0f);

        m3508_right.config_mech_param(48.26f, 0.0f);
        m3508_right.angle_pid_control.ConfigAll(3.1f, 0.4f, 1.4f, 0.0f, 160.0f, 0.2f, 3.0f);

        chassis_3.config(chassis_info_3);
        chassis_3.add_6_motors(&m3508_front, &vesc_front, &m3508_right, &vesc_right, &m3508_left, &vesc_left);

        xbox3.init_plan(0.09f, 0.01f);
        xbox3.addport(&esp);
        xbox3.add_chassis(&chassis_3);
        task_core.registerTask(1, &vesc_front);
        task_core.registerTask(1, &vesc_left);
        task_core.registerTask(1, &vesc_right);
        task_core.registerTask(0, &can);
        task_core.registerTask(3, &xbox3);
        task_core.registerTask(2, &chassis_3);

        osKernelStart();
    }
}

#include "Wch_setup.h"

TaskManager task_core;
CanManager can_core;
RC9Protocol esp_port(uart, &huart2), debug_port(uart, &huart5);
m3508p shooter(3, &hcan2), pitcher(1, &hcan2, true), lifter(2, &hcan2, true), turnner(4, &hcan2); 
vesc vesc1(1, &hcan2), vesc2(2, &hcan2), vesc3(3, &hcan2), m8080(4, &hcan2, 7.0f, 1.0f);

m3508p m3508_left(4, &hcan1, true), m3508_front(3, &hcan1, true), m3508_right(1, &hcan1, true);
vesc vesc_front(1, &hcan1, 21.0f, 3.0f),
    vesc_left(2, &hcan1, 21.0f, 3.0f), vesc_right(3, &hcan1, 21.0f, 3.0f);
chassis_info s3_chassis_info = {0.037f, 0.17f, 0.3f, 0.0f, 0.44f, 0.38735f};
RoboChassis s3_chassis(swerve3_chassis);
UserCtrl_xbox xbox_test(8.0f, 6.0f);

extern "C"
{
    void mytest(void)
    {
        can_core.init();

        esp_port.startUartReceiveIT();
        debug_port.initQueue();

        //俯仰电机
        pitcher.start_debug();
        pitcher.addport(&debug_port);
        pitcher.config_mech_param(19.2032f, 35.0f);
        pitcher.distance_pid_control.ConfigAll(4.0f, 0.0f, 0.086f, 0.0f, 430.0f, 1.0f, 0.0f);

        xbox_test.addport(&esp_port);
        xbox_test.add_motor(&pitcher, &lifter, &m8080, &turnner);
        xbox_test.add_trigger(GPIOC, GPIO_PIN_15, GPIOC, GPIO_PIN_13); // shooter c 13
        
        m3508_front.config_mech_param(48.26f, 0.0f);
        m3508_front.angle_pid_control.ConfigAll(3.1f, 0.4f, 1.4f, 0.0f, 160.0f, 0.2f, 3.0f);

        m3508_left.config_mech_param(48.26f, 0.0f);
        m3508_left.angle_pid_control.ConfigAll(3.1f, 0.4f, 1.4f, 0.0f, 160.0f, 0.2f, 3.0f);

        m3508_right.config_mech_param(48.26f, 0.0f);
        m3508_right.angle_pid_control.ConfigAll(3.1f, 0.4f, 1.4f, 0.0f, 160.0f, 0.2f, 3.0f);

        s3_chassis.config(s3_chassis_info);
        s3_chassis.add_6_motors(&m3508_front, &vesc_front, &m3508_right, &vesc_right, &m3508_left, &vesc_left);

        s3_chassis.add_photogate(GPIOF, GPIO_PIN_11, GPIOF, GPIO_PIN_12, GPIOF, GPIO_PIN_13, GPIOF, GPIO_PIN_10);

        xbox_test.init_plan(0.09f, 0.01f);
        xbox_test.add_chassis(&s3_chassis);

        task_core.registerTask(0, &can_core);
        task_core.registerTask(1, &vesc_front);
        task_core.registerTask(1, &vesc_left);
        task_core.registerTask(1, &vesc_right);
        task_core.registerTask(2, &s3_chassis);
        task_core.registerTask(3, &xbox_test);
        task_core.registerTask(8, &debug_port);

        osKernelStart();
    }
}

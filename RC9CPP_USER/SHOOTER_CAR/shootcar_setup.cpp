#include "shootcar_setup.h"
m6020s m6020_left_front(1, &hcan1), m6020_right_front(2, &hcan1), m6020_left_back(3, &hcan1), m6020_right_back(4, &hcan1); // 舵向电机

CanManager can_core;
TaskManager task_core;

m3508p lifter(1, &hcan2), pitcher(2, &hcan2); // 抬升电机，俯仰电机

vesc vesc_left_front(1, &hcan2), vesc_right_front(2, &hcan2), vesc_left_back(3, &hcan2), vesc_right_back(4, &hcan2); // 航向电调

RC9Protocol esp_port(&huart2, false), pc_port(&huart4, false); // 串口通信

action Action(&huart3, 0.0f, 0.0f, false);

swerve4 shooter_chassis(&Action, 0.2739f, 0.2739f); // 0.0365m轮子半径,0.2739m底盘半径
shoot_xbox box_test(&lifter, &pitcher, &pitcher, &shooter_chassis);
demo test2;

extern "C" void shootcar_setup()
{
    can_core.init();

    Action.startUartReceiveIT();
    esp_port.startUartReceiveIT();
    pc_port.startUartReceiveIT();
    esp_port.addsubscriber(&box_test);
    // m6020_left_back.set_init_angle(120.0f);
    shooter_chassis.add_heading_motor(&m6020_right_front, &m6020_right_back, &m6020_left_back, &m6020_left_front);

    shooter_chassis.add_speed_motor(&vesc_right_front, &vesc_right_back, &vesc_left_back, &vesc_left_front);
    task_core.registerTask(0, &can_core);
    task_core.registerTask(1, &vesc_left_front);
    task_core.registerTask(1, &vesc_right_front);
    task_core.registerTask(1, &vesc_left_back);
    task_core.registerTask(1, &vesc_right_back);
    task_core.registerTask(7, &test2);
    task_core.registerTask(2, &shooter_chassis);
    task_core.registerTask(3, &box_test);

    osKernelStart();
}
void demo::process_data()
{
    // m6020_left_back.set_init_angle(120.0f);
    // vesc_left_front.set_rpm(0.0f);
}
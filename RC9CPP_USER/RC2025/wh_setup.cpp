//
// Created by 15828 on 2025/3/20.
//

#include "wh_setup.h"

TaskManager task_core;
CanManager can_core;
RC9Protocol esp_port(uart, &huart2);


m3508p lifter(1, &hcan1, true),
       turnner(2, &hcan1),
       pithcer(3, &hcan1);

vesc vesc_front(1, &hcan2, 21.0f, 3.0f),
     vesc_left(2, &hcan2, 21.0f, 3.0f),
     vesc_right(3, &hcan2, 21.0f, 3.0f);
vesc m8080(4, &hcan2, 7.0f, 1.0f);

Encoder  wh_encoder(&huart6);
//Shoot_cal demo;
wh_xbox my_xbox;
extern "C"
{
void wh_setup()
{
    can_core.init();
    esp_port.startUartReceiveIT();

	my_xbox.xbox_init();
    my_xbox.addport(&esp_port);
    my_xbox.load_pin(GPIOC,GPIO_PIN_14,GPIOC,GPIO_PIN_13,GPIOC,GPIO_PIN_15);
    my_xbox.load_motor(&lifter, &turnner, &m8080, &pithcer);

    task_core.registerTask(0, &can_core);
    task_core.registerTask(3, &my_xbox);
    task_core.registerTask(1, &m8080);

    m8080.rpm_control.config_all(230.0f, 2.2f, 486.0f, 0.0f, 50000.0f, 6.0f);

    osKernelStart();
}
}

//void demo::process_data()
//{
//    Now_distance
//}
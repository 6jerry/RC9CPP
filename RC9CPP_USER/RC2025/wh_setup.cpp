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
//vesc m8080(4, &hcan2, 7.0f, 1.0f);

wh_xbox my_xbox;

void wh_setup()
{
    esp_port.startUartReceiveIT();
    can_core.init();

    task_core.registerTask(0, &can_core);
    task_core.registerTask(3, &my_xbox);
//    task_core.registerTask(1, &m8080);


    osKernelStart();
}
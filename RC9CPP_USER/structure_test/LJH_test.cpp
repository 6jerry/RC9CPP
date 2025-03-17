#include "LJH_test.h"

TaskManager task;
CanManager can;
RC9Protocol ESP(uart, &huart2);

m3508p motor_1(1,&hcan2), motor_2(2,&hcan2), motor_3(3,&hcan2), motor_4(4,&hcan2);
//vesc motor_5(1,&hcan1), motor_6(2,&hcan1), motor_7(3,&hcan1);

xbox_controller xbox_test;

extern "C"
{
    void LJH_test_setup()
    {
        ESP.startUartReceiveIT();
        can.init();
        task.registerTask(0, &can);
        xbox_test.addport(&ESP);
        xbox_test.add_motor(&motor_1, &motor_2, &motor_3, &motor_4);
        xbox_test.add_trigger(GPIOC, GPIO_PIN_15, GPIOC, GPIO_PIN_13);
        task.registerTask(0, &can);
        task.registerTask(2, &xbox_test);


        osKernelStart();
    }
}
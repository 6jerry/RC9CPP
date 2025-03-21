#include "LJH_test.h"

TaskManager task;
CanManager can;
RC9Protocol ESP(uart, &huart2), debug_port(uart, &huart5);

m3508p motor_1(1,&hcan1,true), motor_2(2,&hcan1), motor_3(3,&hcan1), motor_4(4,&hcan1);
//vesc motor_5(1,&hcan1), motor_6(2,&hcan1), motor_7(3,&hcan1);

xbox_controller xbox_test;

extern "C"
{
    void LJH_test_setup()
    {
        ESP.startUartReceiveIT();
        can.init();
        debug_port.initQueue();
        task.registerTask(0, &can);
        xbox_test.addport(&ESP);
        xbox_test.add_motor(&motor_1, &motor_2, &motor_3, &motor_4);
        xbox_test.add_trigger(GPIOC, GPIO_PIN_15, GPIOC, GPIO_PIN_13);
        motor_1.start_debug();
				motor_1.addport(&debug_port);
			  motor_1.config_mech_param(19.2032f, 35.0f);
        task.registerTask(0, &can);
        task.registerTask(2, &xbox_test);


        osKernelStart();
    }
}
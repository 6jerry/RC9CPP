#include "R3_test.h"

TaskManager task_core;
dji_motor_handle dji_core;

RC9Protocol esp_port(uart, &huart3);
RC9Protocol send_port(uart, &huart5);

vesc shoot_1(vesc_id_1, &hfdcan1, 7.0f, 1.0f);
vesc shoot_2(vesc_id_2, &hfdcan1, 7.0f, 1.0f);

R3_xbox test_xbox;

//demo plot;
extern "C"
{
    void r3_setup()
    {
        CanDevice::InitAllFiltersNoMask();
        esp_port.initQueue();
        esp_port.startUartReceiveIT();

        test_xbox.addport(&esp_port);
        //test_xbox.gate.add_flag(&test_xbox.flag);
			  test_xbox.gate.add_io_interrupt(GPIOD,GPIO_PIN_14);
				//test_xbox.add_io(GPIOD,GPIO_PIN_14);
        test_xbox.shoot_motor_1 = &shoot_1;
        test_xbox.shoot_motor_2 = &shoot_2;
        task_core.registerTask(0, &dji_core);
        task_core.registerTask(2, &shoot_1);
        task_core.registerTask(2, &shoot_2);
			  //task_core.registerTask(2, &plot);
			
        task_core.registerTask(4, &test_xbox);
			
			
        osKernelStart();
    }
}

void demo::process_data()
{
	   //float data[2] = {shoot_1.get_rpm(),shoot_2.get_rpm()};
	  // sendFloatData(1,data,2);

	
}
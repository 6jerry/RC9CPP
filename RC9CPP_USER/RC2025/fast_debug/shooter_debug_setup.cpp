#include "shooter_debug_setup.h"

TaskManager task_core;
dji_motor_handle dji_core;
m3508p left(dji_id_4, &hfdcan1, 111.72384f, M2006_MAX_CURRENT, M2006_CURRENT_MAP), m2006_right(dji_id_1, &hfdcan1, 111.72384f, M2006_MAX_CURRENT, M2006_CURRENT_MAP), m2006_front(dji_id_3, &hfdcan3, 111.72384f, M2006_MAX_CURRENT, M2006_CURRENT_MAP);

//vesc u8_front(vesc_id_1, &hfdcan2), u8_left(vesc_id_2, &hfdcan2), u8_right(vesc_id_3, &hfdcan2);
vesc m6374 (vesc_id_4, &hfdcan2);
RC9Protocol esp_port(uart, &huart3);
Encoder encoder(0x01,&hfdcan3);
Shooter_debug xbox;

AutoShooter auto_shooter;

extern "C"
{

    void shooter_debug_setup()
    {
        CanDevice::InitAllFiltersNoMask();
        esp_port.initQueue();

        esp_port.startUartReceiveIT();

			  auto_shooter.add_encoder(&encoder);
			  auto_shooter.add_motor(&m6374);
			
        xbox.addport(&esp_port);
        xbox.add_auto_shooter(&auto_shooter);

      
        // pid config

        task_core.registerTask(0, &dji_core);
        task_core.registerTask(1, &m6374);
        task_core.registerTask(2, &auto_shooter);
			
        task_core.registerTask(6, &xbox);

        osKernelStart();
    }
}
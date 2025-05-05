#include "omni4_test.h"

TaskManager task_core;
CanManager can_core;
RC9Protocol esp_port(uart, &huart2), position_imu_uart(uart, &huart5);

/*************************************************************************/
action action_imu(&huart4, 0.0f, 18.9f, false);
position position_imu;
RC9Protocol lora_port(uart, &huart3);
Lora lora;

/*************************************************************************/

m3508p front_left_motor(2, &hcan1), front_right_motor(3, &hcan1), back_left_motor(1, &hcan1), back_right_motor(4, &hcan1);

chassis_info omni4_info = {0.0719f, 0.0f, 0.0f, 0.2425f};

RoboChassis chassis_omni4_(omni4_chassis);

auto_lock_test omni4_xbox;

extern "C"
{
    void omni4_test(void)
    {
        can_core.init();
        esp_port.startUartReceiveIT();
        position_imu_uart.startUartReceiveIT();

        lora_port.startUartReceiveIT();
        lora_port.initQueue();
        //debug_port.initQueue();

        chassis_omni4_.config(omni4_info);
        chassis_omni4_.add4_motors(&front_left_motor, &front_right_motor, &back_right_motor, &back_left_motor);

        //chassis_omni4_.enable_debug();

        // chassis_omni4_.add_imu(&action_imu);
        // chassis_omni4_.addport(&debug_port);
        chassis_omni4_.yawadjuster_config(0.029f, 0.0f, 0.002f, 0.0f, 2.0f, 0.2f, 0.0f);
        chassis_omni4_.add_imu(&position_imu);
        // chassis_omni4_.pointtrack_config(0.76f, 0.0f, 0.25f, 0.0f, 5.0f, 0.008f, 0.0f);
        // chassis_omni4_.pp_tracker.normal_control.ConfigAll(2.8f, 0.0f, 0.2f, 0.0f, 5.0f, 0.002f, 0.0f);
        // chassis_omni4_.pp_tracker.tangent_control.ConfigAll(1.2f, 0.0f, 3.8f, 0.0f, 1.5f, 0.002f, 0.0f);

        task_core.customize(4, osPriorityRealtime, 10, 20 * 128);
        omni4_xbox.addport(&esp_port);
        omni4_xbox.add_chassis(&chassis_omni4_);
        position_imu.addport(&position_imu_uart);

        /*************************************************************************/
        lora.addport(&lora_port);
        lora.add_imu(&action_imu);

        task_core.registerTask(5, &lora);
        task_core.registerTask(6, &lora_port);
        /*************************************************************************/

        task_core.registerTask(0, &can_core);
        task_core.registerTask(3, &chassis_omni4_);
        task_core.registerTask(2, &omni4_xbox);
        //task_core.registerTask(8, &position_imu_uart);

        osKernelStart();
    }
}
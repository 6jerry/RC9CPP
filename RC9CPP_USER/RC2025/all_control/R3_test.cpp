#include "R3_test.h"

TaskManager task_core;
dji_motor_handle dji_core;

RC9Protocol esp_port(uart, &huart3), debug_port(uart, &huart4);

vesc shoot_1(vesc_id_1, &hfdcan1, 7.0f, 1.0f);
vesc shoot_2(vesc_id_2, &hfdcan1, 7.0f, 1.0f);

m3508p front_left_motor(dji_id_3, &hfdcan3), front_right_motor(dji_id_4, &hfdcan3), back_left_motor(dji_id_2, &hfdcan3), back_right_motor(dji_id_1, &hfdcan3);
RoboChassis robot_chassis(omni4_chassis);
chassis_info omni4_info = {0.0719f, 0.0f, 0.0f, 0.2425f, 0.0f, 0.0f};

R3_xbox test_xbox;
demo plot;

// demo plot;
extern "C"
{
    void r3_setup()
    {
        CanDevice::InitAllFiltersNoMask();
        esp_port.initQueue();
        esp_port.startUartReceiveIT();
        debug_port.startUartReceiveIT();
        debug_port.initQueue();
			front_left_motor.rpm_control.config_all(32.0f, 0.76f, 8.6f, 106.0f, 20000.0f, 5.0f);
			front_right_motor.rpm_control.config_all(32.0f, 0.76f, 8.6f, 106.0f, 20000.0f, 5.0f);
			 back_left_motor.rpm_control.config_all(32.0f, 0.76f, 8.6f, 106.0f, 20000.0f, 5.0f);
			back_right_motor.rpm_control.config_all(32.0f, 0.76f, 8.6f, 106.0f, 20000.0f, 5.0f);
        test_xbox.addport(&esp_port);
        test_xbox.gate.add_io_interrupt(GPIOD, GPIO_PIN_14);
        test_xbox.add_chassis(&robot_chassis);
        robot_chassis.add4_motors(&front_left_motor, &front_right_motor, &back_left_motor, &back_right_motor);
        robot_chassis.config(omni4_info);
        robot_chassis.pointtrack_config(0.76f, 0.0f, 0.25f, 0.0f, 5.0f, 0.008f, 0.0f);
        // omi4_chassis.add_imu(&position_sensor);
        robot_chassis.yawadjuster_config(0.12f, 0.0f, 0.004f, 0.0f, 5.0f, 0.1f, 0.5f);

        shoot_1.rpm_control.config_all(70.0f, 1.0f, 140.0f, 20.0f, 65000, 5.0f);

        shoot_2.rpm_control.config_all(70.0f, 1.0f, 140.0f, 20.0f, 65000, 5.0f);
        shoot_1.rpm_control.enable_TD();
        shoot_2.rpm_control.enable_TD();
        // shoot_1.addport(&debug_port);
        // shoot_1.start_debug();

        plot.addport(&debug_port);

        test_xbox.shoot_motor_1 = &shoot_1;
        test_xbox.shoot_motor_2 = &shoot_2;
        task_core.registerTask(0, &dji_core);
        task_core.registerTask(2, &shoot_1);
        task_core.registerTask(2, &shoot_2);
        task_core.registerTask(4, &robot_chassis);
        task_core.registerTask(6, &test_xbox);
        task_core.registerTask(8, &debug_port);
        task_core.registerTask(2, &plot);
        osKernelStart();
    }
}

void demo::process_data()
{
    float send_datas[3] = {shoot_1.get_rpm(),
                           shoot_2.get_rpm(),
                           test_xbox.target_rpm};

    sendFloatData(1, send_datas, 3);
}
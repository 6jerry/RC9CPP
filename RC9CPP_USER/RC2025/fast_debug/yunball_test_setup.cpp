#include "yunball_test_setup.h"

TaskManager task_core;
dji_motor_handle dji_core;

//电机
m3508p turn_motor(dji_id_2, &hfdcan1, 49.1372f), m2006_left(dji_id_4, &hfdcan1, 111.72384f, M2006_MAX_CURRENT, M2006_CURRENT_MAP), m2006_right(dji_id_1, &hfdcan1, 111.72384f, M2006_MAX_CURRENT, M2006_CURRENT_MAP), m2006_front(dji_id_3, &hfdcan1, 111.72384f, M2006_MAX_CURRENT, M2006_CURRENT_MAP);
vesc u8_front(vesc_id_1, &hfdcan2), u8_left(vesc_id_2, &hfdcan2), u8_right(vesc_id_3, &hfdcan2);

//串口
RC9Protocol esp_port(uart, &huart3), position_port(uart, &huart6);

//类应用
position position_sensor;
yunball_test_xbox xbox(&position_sensor);
RoboChassis s3_chassis(swerve3_chassis);
chassis_info s3_chassis_info = {0.037f, 0.17f, 0.3f, 0.0f, 0.44f, 0.38735f};
auto_yunball yunball_port;

extern "C" {
    void yunball_test_setup()
    {
        //xbox
        CanDevice::InitAllFiltersNoMask();
        esp_port.initQueue();
        esp_port.startUartReceiveIT();
        xbox.addport(&esp_port);
        xbox.add_autoyunball(&yunball_port);
        xbox.add_chassis(&s3_chassis);

        //position
        position_port.initQueue();
        position_port.startUartReceiveIT();
        position_sensor.addport(&position_port);

        //运球
				yunball_port.add_motor(&turn_motor);
				yunball_port.add_io(GPIOG, GPIO_PIN_6, GPIOG, GPIO_PIN_8, GPIOG, GPIO_PIN_5);   //7发射， 8夹爪，6抬升， 5推射
			
        //底盘
        s3_chassis.add_6_motors(&m2006_front, &u8_front, &m2006_right, &u8_right, &m2006_left, &u8_left);
        s3_chassis.config(s3_chassis_info);
        s3_chassis.yawadjuster_config(0.1f, 0.0f, 0.03f, 0.0f, 7.0f, 0.2f, 0.0f);
        s3_chassis.pointtrack_config(0.76f, 0.0f, 0.25f, 0.0f, 5.0f, 0.008f, 0.0f);
        s3_chassis.add_imu(&position_sensor);
        s3_chassis.add_photogate(GPIOF, GPIO_PIN_8, GPIOF, GPIO_PIN_9, GPIOD, GPIO_PIN_15, nullptr, 0);

        // pid config
        m2006_left.rpm_control.config_all(12.0f, 0.9f, 8.6f, 0.0f, 10000.0f, 3.0f);
        m2006_left.angle_pid_control.ConfigAll(3.6f, 0.0f, 6.0f, 0.0f, 120.0f, 0.2f, 3.0f);
        m2006_front.rpm_control.config_all(12.0f, 0.9f, 8.6f, 0.0f, 10000.0f, 3.0f);
        m2006_front.angle_pid_control.ConfigAll(3.6f, 0.0f, 6.0f, 0.0f, 120.0f, 0.2f, 3.0f);
        m2006_right.rpm_control.config_all(12.0f, 0.9f, 8.6f, 0.0f, 10000.0f, 3.0f);
        m2006_right.angle_pid_control.ConfigAll(3.6f, 0.0f, 6.0f, 0.0f, 120.0f, 0.2f, 3.0f);
        turn_motor.rpm_control.config_all(32.0f, 0.76f, 8.6f, 106.0f, 20000.0f, 5.0f);

        // task register
        task_core.registerTask(0, &dji_core);
        task_core.registerTask(1, &u8_front);
        task_core.registerTask(1, &u8_left);
        task_core.registerTask(1, &u8_right);
        task_core.registerTask(4, &s3_chassis);
        task_core.registerTask(7, &position_port);
        task_core.registerTask(2, &xbox);
				task_core.registerTask(5, &yunball_port);
        osKernelStart();
    }
}
#include "all_setup.h"
TaskManager task_core;
dji_motor_handle dji_core;

m3508p m2006_left(dji_id_4, &hfdcan1, 111.72384f, M2006_MAX_CURRENT, M2006_CURRENT_MAP), m2006_right(dji_id_1, &hfdcan1, 111.72384f, M2006_MAX_CURRENT, M2006_CURRENT_MAP), m2006_front(dji_id_3, &hfdcan1, 111.72384f, M2006_MAX_CURRENT, M2006_CURRENT_MAP);

vesc u8_front(vesc_id_1, &hfdcan2), u8_left(vesc_id_2, &hfdcan2), u8_right(vesc_id_3, &hfdcan2);

RoboChassis s3_chassis(swerve3_chassis);
chassis_info s3_chassis_info = {0.037f, 0.17f, 0.3f, 0.0f, 0.44f, 0.38735f};

CrsfReceiver remote_controller(&huart7);
RC9Protocol position_port(uart, &huart4);
position position_sensor;
demo plot;
extern "C"
{
    void all_setup()
    {
        CanDevice::InitAllFiltersNoMask();
        position_port.initQueue();
        position_port.startUartReceiveIT();

        remote_controller.startUartReceiveIT();
        position_sensor.addport(&position_port);
        s3_chassis.add_6_motors(&m2006_front, &u8_front, &m2006_right, &u8_right, &m2006_left, &u8_left);
        s3_chassis.config(s3_chassis_info);
        s3_chassis.pointtrack_config(0.76f, 0.0f, 0.25f, 0.0f, 5.0f, 0.008f, 0.0f);
        s3_chassis.add_imu(&position_sensor);
        s3_chassis.add_photogate(GPIOF, GPIO_PIN_8, GPIOF, GPIO_PIN_9, GPIOD, GPIO_PIN_15, nullptr, 0);
        s3_chassis.yawadjuster_config(0.12f, 0.0f, 0.004f, 0.0f, 5.0f, 0.1f, 0.5f);
        // remote_controller.add_chassis(&s3_chassis);

        // pid config
        m2006_left.rpm_control.config_all(12.0f, 0.9f, 8.6f, 0.0f, 10000.0f, 3.0f);
        m2006_left.angle_pid_control.ConfigAll(3.6f, 0.0f, 6.0f, 0.0f, 120.0f, 0.2f, 3.0f);
        m2006_front.rpm_control.config_all(12.0f, 0.9f, 8.6f, 0.0f, 10000.0f, 3.0f);
        m2006_front.angle_pid_control.ConfigAll(3.6f, 0.0f, 6.0f, 0.0f, 120.0f, 0.2f, 3.0f);
        m2006_right.rpm_control.config_all(12.0f, 0.9f, 8.6f, 0.0f, 10000.0f, 3.0f);
        m2006_right.angle_pid_control.ConfigAll(3.6f, 0.0f, 6.0f, 0.0f, 120.0f, 0.2f, 3.0f);

        task_core.registerTask(0, &dji_core);
        //        task_core.registerTask(2, &u8_front);
        //        task_core.registerTask(2, &u8_left);
        //        task_core.registerTask(2, &u8_right);
        //        task_core.registerTask(4, &s3_chassis);
        //        task_core.registerTask(5, &remote_controller);
        task_core.registerTask(8, &plot);
        osKernelStart();
    }
}

void demo::process_data()
{
    //remote_controller.sendAttitude(1.0f, 1.0f, 1.0f);
    // remote_controller.sendBattery(test_v, test_c, 1.0f, 1);

    remote_controller.sendGps(1.0, 1.0, 3, 3, 7, 12);
}
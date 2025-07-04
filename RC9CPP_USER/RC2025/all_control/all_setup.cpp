#include "all_setup.h"
TaskManager task_core;
dji_motor_handle dji_core;

m3508p m2006_left(dji_id_4, &hfdcan1, 111.72384f, M2006_MAX_CURRENT, M2006_CURRENT_MAP), m2006_right(dji_id_1, &hfdcan1, 111.72384f, M2006_MAX_CURRENT, M2006_CURRENT_MAP), m2006_front(dji_id_3, &hfdcan1, 111.72384f, M2006_MAX_CURRENT, M2006_CURRENT_MAP), turn_motor(dji_id_2, &hfdcan1, 49.1372f), lift_motor(dji_id_5, &hfdcan1);

vesc u8_front(vesc_id_1, &hfdcan2), u8_left(vesc_id_2, &hfdcan2), u8_right(vesc_id_3, &hfdcan2), m6374(vesc_id_4, &hfdcan2, 7.0f, 2.0f);

RoboChassis s3_chassis(swerve3_chassis);
chassis_info s3_chassis_info = {0.037f, 0.17f, 0.3f, 0.0f, 0.44f, 0.38735f};

CrsfReceiver remote_controller(&huart2);

AllController control_center;

Encoder encoder(0x01, &hfdcan3);

RC9Protocol position_port(uart, &huart6), Lora_port(uart, &huart5), ros_port(cdc, nullptr);
position position_sensor;
ros_sensor ros_sensor_;

AutoShooter auto_shooter;

auto_yunball yunball_port;
demo plot;
extern "C"
{
    void all_setup()
    {
        CanDevice::InitAllFiltersNoMask();
        position_port.initQueue();
        position_port.startUartReceiveIT();
        Lora_port.startUartReceiveIT();
        Lora_port.initQueue();
        remote_controller.startUartReceiveIT();
        position_sensor.addport(&position_port);

        ros_sensor_.add_recolate_imu(&position_sensor);
        ros_sensor_.addport(&ros_port);
        ros_port.initQueue();
        s3_chassis.add_6_motors(&m2006_front, &u8_front, &m2006_right, &u8_right, &m2006_left, &u8_left);
        s3_chassis.config(s3_chassis_info);
        // s3_chassis.pointtrack_config(0.76f, 0.0f, 0.25f, 0.0f, 5.0f, 0.008f, 0.0f);
        s3_chassis.add_imu(&position_sensor);
        s3_chassis.add_photogate(GPIOF, GPIO_PIN_8, GPIOF, GPIO_PIN_9, GPIOD, GPIO_PIN_15, nullptr, 0);
        s3_chassis.yawadjuster_config(0.12f, 0.0f, 0.004f, 0.0f, 5.0f, 0.1f, 0.5f);
        // remote_controller.add_chassis(&s3_chassis);
        control_center.add_elrs(&remote_controller);
        control_center.add_chassis(&s3_chassis);
        control_center.addport(&Lora_port);
        control_center.add_yunball_and_shooter(&auto_shooter, &yunball_port);

        control_center.add_position_and_ros(&position_sensor, &ros_sensor_);
        lift_motor.config_mech_param(19.2032f, 1.0f);
        yunball_port.add_motor(&turn_motor, &lift_motor);
        yunball_port.add_io(GPIOG, GPIO_PIN_6, GPIOG, GPIO_PIN_3, GPIOD, GPIO_PIN_14); // 8发射， 6夹爪， 3推射
        yunball_port.add_shooter(&auto_shooter);

        // pid config
        m2006_left.rpm_control.config_all(12.0f, 0.9f, 8.6f, 0.0f, 10000.0f, 3.0f);
        m2006_left.angle_pid_control.ConfigAll(3.6f, 0.0f, 6.0f, 0.0f, 120.0f, 0.2f, 3.0f);
        m2006_front.rpm_control.config_all(12.0f, 0.9f, 8.6f, 0.0f, 10000.0f, 3.0f);
        m2006_front.angle_pid_control.ConfigAll(3.6f, 0.0f, 6.0f, 0.0f, 120.0f, 0.2f, 3.0f);
        m2006_right.rpm_control.config_all(12.0f, 0.9f, 8.6f, 0.0f, 10000.0f, 3.0f);
        m2006_right.angle_pid_control.ConfigAll(3.6f, 0.0f, 6.0f, 0.0f, 120.0f, 0.2f, 3.0f);

        turn_motor.rpm_control.config_all(32.0f, 0.76f, 8.6f, 106.0f, 20000.0f, 5.0f);

        lift_motor.rpm_control.config_all(32.0f, 0.76f, 8.6f, 106.0f, 20000.0f, 5.0f);

        // auto_shooter
        auto_shooter.add_encoder(&encoder);
        auto_shooter.add_motor(&m6374);
        auto_shooter.dis_control.ConfigAll(16000.0f, 3.3f, 64.0f, 0.0f, 1800.0f, 0.001f, 0.015f);
        auto_shooter.add_plan_info(400, 400, 1200, 400, 400);
        auto_shooter.add_trigger(GPIOF, GPIO_PIN_5, GPIOG, GPIO_PIN_8);

        task_core.registerTask(0, &dji_core);
        task_core.registerTask(2, &u8_front);
        task_core.registerTask(2, &u8_left);
        task_core.registerTask(2, &u8_right);
        task_core.registerTask(4, &s3_chassis);
        task_core.registerTask(5, &control_center);
        task_core.registerTask(1, &m6374);
        task_core.registerTask(6, &yunball_port);
        task_core.registerTask(9, &ros_port);
        task_core.registerTask(2, &auto_shooter);
        task_core.registerTask(8, &position_port);
        task_core.registerTask(8, &Lora_port);
        //task_core.registerTask(8, &plot);
        osKernelStart();
    }
}

void demo::process_data()
{
    //remote_controller.sendAttitude(test_v, test_v, test_v);
    //remote_controller.sendBattery(test_v, test_v, test_v, test_v);

    //remote_controller.sendGps(test_x, test_y, test_heading, test_heading, test_heading, test_heading);
    
}
#include "all_setup_r3.h"

TaskManager task_core;
dji_motor_handle dji_core;

RC9Protocol position_port(uart, &huart6);
RC9Protocol ros_port(cdc, nullptr);
RC9Protocol send_port(uart, &huart4);

ros_sensor ros_sensor_;
position position_sensor;
Camera camera;
CameraOperation camera_operation(&camera);

vesc shoot_1(vesc_id_4, &hfdcan1, 7.0f, 1.0f), shoot_2(vesc_id_5, &hfdcan1, 7.0f, 1.0f);
vesc u8_front(vesc_id_1, &hfdcan3), u8_left(vesc_id_2, &hfdcan3), u8_right(vesc_id_3, &hfdcan3);
m3508p m2006_putball(dji_id_1, &hfdcan2, 55.4248f, M2006_MAX_CURRENT, M2006_CURRENT_MAP);
m3508p m2006_left(dji_id_4, &hfdcan2, 111.72384f, M2006_MAX_CURRENT, M2006_CURRENT_MAP), m2006_front(dji_id_3, &hfdcan2, 111.72384f, M2006_MAX_CURRENT, M2006_CURRENT_MAP), m2006_right(dji_id_2, &hfdcan2, 111.72384f, M2006_MAX_CURRENT, M2006_CURRENT_MAP);

RoboChassis s3_chassis(swerve3_chassis);
chassis_info s3_chassis_info = {0.037f, 0.17f, 0.3f, 0.0f, 0.44f, 0.38735f};

Encoder encoder(0x03, &hfdcan2, 2.0f, 4096.0f, 0.175f);

AutoYunballR3 auto_yunball;
R3Shooter shooter;
photogate_shoot gate(rising, GPIOD, GPIO_PIN_14);
photogate_shoot gate_down(falling, GPIOE, GPIO_PIN_3);

demo plot;

CrsfReceiver remote_controller(&huart2);

R3Controller control_center;

error_manager error_core;

extern "C"
{
    void r3_all_setup()
    {
        CanDevice::InitAllFiltersNoMask();
        time_counter::init_time_counter();

        // UART5
        send_port.initQueue();
        send_port.startUartReceiveIT();
        // plot.addport(&send_port);
        // plot.add_xbox(&test_xbox);

        // LiDAR
        ros_sensor_.add_recolate_imu(&position_sensor);
        ros_sensor_.addport(&ros_port);
        ros_port.initQueue();
        ros_port.startUartReceiveIT();

        // camera
        camera.addport(&ros_port);
        camera_operation.add_chassis(&s3_chassis);
        control_center.add_camera(&camera_operation);

        // position
        position_port.initQueue();
        position_port.startUartReceiveIT();
        position_sensor.addport(&position_port);
        position_sensor.set_map_plot(0.0f, -0.08229f);
        position_sensor.position_EC.config_param(3, 100, 2000);

        remote_controller.startUartReceiveIT();
        remote_controller.pocket_ec.config_param(0, 100, 3000);

        // DJI_Motor pid config
        m2006_putball.config_mech_param(55.4248f, 2.0f);
        m2006_putball.rpm_control.config_all(12.0f, 0.9f, 8.6f, 0.0f, 10000.0f, 3.0f);
        m2006_putball.m3508_ec.config_param(9, 20, 1000);

        m2006_left.rpm_control.config_all(12.0f, 0.9f, 8.6f, 0.0f, 10000.0f, 3.0f);

        m2006_left.m3508_ec.config_param(12, 20, 1000);

        m2006_front.rpm_control.config_all(12.0f, 0.9f, 8.6f, 0.0f, 10000.0f, 3.0f);

        m2006_front.m3508_ec.config_param(11, 20, 1000);

        m2006_right.rpm_control.config_all(12.0f, 0.9f, 8.6f, 0.0f, 10000.0f, 3.0f);
        m2006_right.m3508_ec.config_param(10, 20, 1000);

        m2006_front.angle_pid_control.ConfigAll(3.6f, 0.0f, 6.0f, 0.0f, 120.0f, 0.2f, 3.0f);

        m2006_left.angle_pid_control.ConfigAll(3.6f, 0.0f, 6.0f, 0.0f, 120.0f, 0.2f, 3.0f);

        m2006_right.angle_pid_control.ConfigAll(3.6f, 0.0f, 6.0f, 0.0f, 120.0f, 0.2f, 3.0f);

        // vesc pid config
        shoot_1.rpm_control.config_all(70.0f, 1.0f, 140.0f, 20.0f, 65000, 5.0f);
        shoot_1.add_slave_motor(&shoot_2);
        shoot_1.rpm_control.enable_TD();
        shoot_2.rpm_control.enable_TD();

        shoot_1.vesc_ec.config_param(7, 200, 2000);
        shoot_2.vesc_ec.config_param(8, 200, 2000);

        encoder.encoder_ec.config_param(1, 100, 2000);

        // yunball & shooter
        auto_yunball.add_motor(&m2006_putball);
        auto_yunball.add_io(GPIOA, GPIO_PIN_8, GPIOA, GPIO_PIN_2, GPIOG, GPIO_PIN_7, GPIOG, GPIO_PIN_6);

        shooter.init(&shoot_1, &shoot_2, &encoder);
        shooter.add_gate(&gate, &gate_down);
        shooter.dis_control.ConfigAll(1400.0f, 0.0f, 83.0f, 0.0f, 1000.0f, 0.005f, 0.015f);
        gate.set_motors(&shoot_1, &shoot_2);
        gate_down.set_motors(&shoot_1, &shoot_2);

        u8_front.vesc_ec.config_param(4, 200, 2000);
        u8_left.vesc_ec.config_param(5, 200, 2000);
        u8_right.vesc_ec.config_param(6, 200, 2000);

        // chassis
        s3_chassis.add_6_motors(&m2006_front, &u8_front, &m2006_right, &u8_right, &m2006_left, &u8_left);
        s3_chassis.config(s3_chassis_info);
        s3_chassis.pointtrack_config(0.76f, 0.0f, 0.25f, 0.0f, 5.0f, 0.008f, 0.0f);
        s3_chassis.add_imu(&position_sensor);
        s3_chassis.add_photogate(GPIOF, GPIO_PIN_8, GPIOF, GPIO_PIN_9, GPIOD, GPIO_PIN_15, nullptr, 0);
        s3_chassis.yawadjuster_config(0.12f, 0.0f, 0.004f, 0.0f, 5.0f, 0.1f, 0.5f);

        control_center.add_elrs(&remote_controller);
        control_center.add_chassis(&s3_chassis);
        control_center.addport(&send_port);
        control_center.add_yunball_and_shooter(&shooter, &auto_yunball);
        control_center.add_position_and_ros(&position_sensor, &ros_sensor_);

        task_core.registerTask(0, &dji_core);
        task_core.registerTask(2, &shoot_1);
        task_core.registerTask(2, &shoot_2);
        task_core.registerTask(1, &u8_front);
        task_core.registerTask(1, &u8_left);
        task_core.registerTask(1, &u8_right);
        task_core.registerTask(3, &shooter);
        task_core.registerTask(4, &s3_chassis);
        task_core.registerTask(5, &ros_port);
        task_core.registerTask(5, &control_center);
        task_core.registerTask(7, &auto_yunball);
        task_core.registerTask(8, &position_port);
        task_core.registerTask(8, &plot);
        task_core.registerTask(9, &camera_operation);
        task_core.registerTask(2, &send_port);
        task_core.registerTask(6, &error_core);

        osKernelStart();
    }
}

void demo::process_data()
{
    MX_FDCAN1_Init();
    MX_FDCAN2_Init();
    MX_FDCAN3_Init();
    CanDevice::InitAllFiltersNoMask();
}

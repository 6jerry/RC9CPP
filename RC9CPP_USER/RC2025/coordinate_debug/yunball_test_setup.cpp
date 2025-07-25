#include "yunball_test_setup.h"

TaskManager task_core;
dji_motor_handle dji_core;
m3508p m2006_turn_motor(dji_id_2, &hfdcan1, 111.72384f, M2006_MAX_CURRENT, M2006_CURRENT_MAP), lift_motor(dji_id_5, &hfdcan1);
m3508p m2006_left(dji_id_4, &hfdcan1, 111.72384f, M2006_MAX_CURRENT, M2006_CURRENT_MAP), m2006_right(dji_id_1, &hfdcan1, 111.72384f, M2006_MAX_CURRENT, M2006_CURRENT_MAP), m2006_front(dji_id_3, &hfdcan1, 111.72384f, M2006_MAX_CURRENT, M2006_CURRENT_MAP);
vesc u8_front(vesc_id_1, &hfdcan2), u8_left(vesc_id_2, &hfdcan2), u8_right(vesc_id_3, &hfdcan2);
// vesc m6374(vesc_id_4, &hfdcan2, 7.0f, 2.0f);

Encoder encoder_for_yunball(0x02, &hfdcan3, 3.0f, 1024.0f); // 用于运球的编码器
RC9Protocol esp_port(uart, &huart3), position_port(uart, &huart6);

chassis_info s3_chassis_info = {0.037f, 0.17f, 0.3f, 0.0f, 0.44f, 0.38735f};
RoboChassis s3_chassis(swerve3_chassis);

position position_sensor;

AutoShooter auto_shooter;
auto_yunball yunball_port;

yunball_test_xbox xbox_port(&position_sensor);

extern "C"
{
  void yunball_test_setup()
  {
    CanDevice::InitAllFiltersNoMask();

    // position
    position_port.initQueue();
    position_port.startUartReceiveIT();
    position_sensor.addport(&position_port);

    //yunball
    lift_motor.config_mech_param(19.2032f, 1.0f);
    yunball_port.add_motor(&m2006_turn_motor, &lift_motor);
    yunball_port.add_io(GPIOG, GPIO_PIN_6, GPIOG, GPIO_PIN_3, GPIOD, GPIO_PIN_14); // G8发射， G6夹爪， G3推射
    yunball_port.add_shooter(&auto_shooter);
    m2006_turn_motor.add_encoder(&encoder_for_yunball);
    m2006_turn_motor.rpm_control.config_all(12.0f, 0.9f, 8.6f, 0.0f, 10000.0f, 3.0f);
    lift_motor.rpm_control.config_all(32.0f, 0.76f, 8.6f, 106.0f, 20000.0f, 5.0f);

    // xbox
    esp_port.initQueue();
    esp_port.startUartReceiveIT();
    xbox_port.addport(&esp_port);
    xbox_port.add_autoyunball(&yunball_port);
    xbox_port.add_chassis(&s3_chassis);

    // auto_shooter
    //        auto_shooter.add_encoder(&encoder);
    //        auto_shooter.add_motor(&m6374);
    //        auto_shooter.dis_control.ConfigAll(16000.0f, 3.3f, 64.0f, 0.0f, 1800.0f, 0.001f, 0.015f);
    //        auto_shooter.add_plan_info(400, 400, 1200, 400, 400);
    //        auto_shooter.add_trigger(GPIOF, GPIO_PIN_5, GPIOG, GPIO_PIN_7);
    //        auto_shooter.add_fitter(&fitter);

    // chassis config
    s3_chassis.config(s3_chassis_info);
    s3_chassis.add_imu(&position_sensor);
    s3_chassis.add_photogate(GPIOF, GPIO_PIN_8, GPIOF, GPIO_PIN_9, GPIOD, GPIO_PIN_15, nullptr, 0);
    s3_chassis.add_6_motors(&m2006_front, &u8_front, &m2006_right, &u8_right, &m2006_left, &u8_left);
    s3_chassis.yawadjuster_config(0.1f, 0.000f, 0.03f, 0.0f, 7.0f, 0.04f, 2.0f);
    s3_chassis.pp_tracker.normal_control.ConfigAll(4.0f, 0.0f, 0.1f, 0.0f, 5.0f, 0.05f, 0.0f);
    s3_chassis.pp_tracker.tangent_control.ConfigAll(1.2f, 0.0f, 3.8f, 0.0f, 1.5f, 0.002f, 0.0f);

    // pid config
    m2006_left.rpm_control.config_all(12.0f, 0.9f, 8.6f, 0.0f, 10000.0f, 3.0f);
    m2006_left.angle_pid_control.ConfigAll(3.6f, 0.0f, 6.0f, 0.0f, 120.0f, 0.2f, 3.0f);
    m2006_front.rpm_control.config_all(12.0f, 0.9f, 8.6f, 0.0f, 10000.0f, 3.0f);
    m2006_front.angle_pid_control.ConfigAll(3.6f, 0.0f, 6.0f, 0.0f, 120.0f, 0.2f, 3.0f);
    m2006_right.rpm_control.config_all(12.0f, 0.9f, 8.6f, 0.0f, 10000.0f, 3.0f);
    m2006_right.angle_pid_control.ConfigAll(3.6f, 0.0f, 6.0f, 0.0f, 120.0f, 0.2f, 3.0f);

    task_core.registerTask(2, &u8_front);
    task_core.registerTask(2, &u8_left);
    task_core.registerTask(2, &u8_right);
    task_core.registerTask(0, &dji_core);
    task_core.registerTask(2, &xbox_port);
    task_core.registerTask(5, &yunball_port);
    // task_core.registerTask(2, &auto_shooter);
    task_core.registerTask(4, &s3_chassis);
    task_core.registerTask(8, &position_port);
    // task_core.registerTask(1, &m6374);
    osKernelStart();
  }
}
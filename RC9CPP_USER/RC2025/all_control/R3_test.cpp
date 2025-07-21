#include "R3_test.h"

TaskManager task_core;
dji_motor_handle dji_core;

RC9Protocol esp_port(uart, &huart3), debug_port(uart, &huart4), position_port(uart, &huart5);
RC9Protocol ros_port(cdc, nullptr);
vesc shoot_1(vesc_id_1, &hfdcan3, 7.0f, 1.0f);
vesc shoot_2(vesc_id_2, &hfdcan3, 7.0f, 1.0f);
m3508p m2006_putball(dji_id_1, &hfdcan2, 55.4248f, M2006_MAX_CURRENT, M2006_CURRENT_MAP);

Encoder encoder(0x03, &hfdcan2, 2.0f, 4096.0f);
photogate_shoot gate(rising, GPIOD, GPIO_PIN_14);
photogate_shoot gate_down(falling, GPIOE, GPIO_PIN_3);
R3Shooter shooter;

// m3508p front_left_motor(dji_id_3, &hfdcan3),
//     front_right_motor(dji_id_4, &hfdcan3), back_left_motor(dji_id_2, &hfdcan3), back_right_motor(dji_id_1, &hfdcan3);

m3508p m2006_left(dji_id_4, &hfdcan1, 32.0f, M2006_MAX_CURRENT, M2006_CURRENT_MAP), m2006_right(dji_id_1, &hfdcan1, 111.72384f, M2006_MAX_CURRENT, M2006_CURRENT_MAP), m2006_front(dji_id_3, &hfdcan1, 111.72384f, M2006_MAX_CURRENT, M2006_CURRENT_MAP);

vesc u8_front(vesc_id_1, &hfdcan2), u8_left(vesc_id_2, &hfdcan2), u8_right(vesc_id_3, &hfdcan2);
RoboChassis s3_chassis(swerve3_chassis);
chassis_info s3_chassis_info = {0.037f, 0.17f, 0.3f, 0.0f, 0.44f, 0.38735f};

ros_sensor ros_sensor_;
position position_sensor;
R3_xbox test_xbox(&position_sensor);
AutoYunballR3 auto_yunball;
demo plot;

// demo plot;
extern "C"
{
  void r3_setup()
  {
    CanDevice::InitAllFiltersNoMask();
    time_counter::init_time_counter();
    esp_port.initQueue();
    esp_port.startUartReceiveIT();
    debug_port.startUartReceiveIT();
    debug_port.initQueue();
    ros_sensor_.addport(&ros_port);
    ros_port.initQueue();
    ros_port.startUartReceiveIT();

    m2006_putball.config_mech_param(55.4248f, 2.0f);
    m2006_putball.rpm_control.config_all(32.0f, 0.76f, 8.6f, 106.0f, 20000.0f, 5.0f);
    position_port.startUartReceiveIT();
    position_port.initQueue();
    // front_left_motor.rpm_control.config_all(32.0f, 0.76f, 8.6f, 106.0f, 20000.0f, 5.0f);
    // front_right_motor.rpm_control.config_all(32.0f, 0.76f, 8.6f, 106.0f, 20000.0f, 5.0f);
    // back_left_motor.rpm_control.config_all(32.0f, 0.76f, 8.6f, 106.0f, 20000.0f, 5.0f);
    // back_right_motor.rpm_control.config_all(32.0f, 0.76f, 8.6f, 106.0f, 20000.0f, 5.0f);

    auto_yunball.add_motor(&m2006_putball);
    auto_yunball.add_io(GPIOA, GPIO_PIN_8, GPIOA, GPIO_PIN_2, GPIOG, GPIO_PIN_7, GPIOG, GPIO_PIN_6);
    shooter.init(&shoot_1, &shoot_2, &encoder);
    shooter.add_gate(&gate, &gate_down);
    gate.set_motors(&shoot_1, &shoot_2);
    gate_down.set_motors(&shoot_1, &shoot_2);

    test_xbox.addport(&esp_port);
    test_xbox.add_R3shooter(&shooter);
    // test_xbox.gate.add_io_interrupt(GPIOD, GPIO_PIN_14);
    // test_xbox.gate_down.add_io_interrupt(GPIOE, GPIO_PIN_3);
    test_xbox.add_chassis(&s3_chassis);
    test_xbox.add_autoyunball(&auto_yunball);
    // robot_chassis.add4_motors(&back_right_motor, &front_right_motor, &front_left_motor, &back_left_motor);
    // robot_chassis.config(omni4_info);
    // s3_chassis.pointtrack_config(0.76f, 0.0f, 0.25f, 0.0f, 5.0f, 0.008f, 0.0f);
    // s3_chassis.add_imu(&position_sensor);
    // s3_chassis.yawadjuster_config(0.12f, 0.0f, 0.004f, 0.0f, 5.0f, 0.1f, 0.5f);

    // chassis
    s3_chassis.add_6_motors(&m2006_front, &u8_front, &m2006_right, &u8_right, &m2006_left, &u8_left);
    s3_chassis.config(s3_chassis_info);
    s3_chassis.pointtrack_config(0.76f, 0.0f, 0.25f, 0.0f, 5.0f, 0.008f, 0.0f);
    s3_chassis.add_imu(&position_sensor);
    s3_chassis.add_photogate(GPIOF, GPIO_PIN_8, GPIOF, GPIO_PIN_9, GPIOD, GPIO_PIN_15, nullptr, 0);
    s3_chassis.yawadjuster_config(0.12f, 0.0f, 0.004f, 0.0f, 5.0f, 0.1f, 0.5f);

    shoot_1.rpm_control.config_all(70.0f, 1.0f, 140.0f, 20.0f, 65000, 5.0f);
    shoot_1.add_slave_motor(&shoot_2);
    shoot_1.rpm_control.enable_TD();
    shoot_2.rpm_control.enable_TD();

    //    shoot_1.addport(&debug_port);
    //    shoot_1.start_debug();

    ros_sensor_.add_recolate_imu(&position_sensor);
    ros_sensor_.addport(&ros_port);
    position_sensor.set_map_plot(0.0f, 0.36562f);
    position_sensor.addport(&position_port);

    plot.addport(&debug_port);
    task_core.registerTask(0, &dji_core);
    task_core.registerTask(2, &u8_front);
    task_core.registerTask(2, &u8_left);
    task_core.registerTask(2, &u8_right);
    task_core.registerTask(2, &shoot_1);
    task_core.registerTask(2, &shoot_2);
    task_core.registerTask(4, &s3_chassis);
    task_core.registerTask(5, &shooter);
    task_core.registerTask(6, &test_xbox);

    task_core.registerTask(2, &plot);
    task_core.registerTask(8, &auto_yunball);
    task_core.registerTask(2, &debug_port);
    task_core.registerTask(7, &position_port);
    task_core.registerTask(5, &ros_port);
    osKernelStart();
  }
}

void demo::process_data()
{

  /*if(test_flag == 1)
  {

    encoder.send_reset();

     test_flag = 0;
  }

  */
  float send_datas[5] = {gate.rpm1,
                         gate.rpm2,
                         test_xbox.target_rpm,
                         shoot_1.get_rpm(),
                         shoot_2.get_rpm()};

  sendFloatData(1, send_datas, 5);
}

void demo::DataReceivedCallback(const uint8_t *byteData, const float *floatData, uint8_t id, uint16_t byteCount)
{
  for (int i = 0; i < 4; i++)
  {
    recive_data[i] = floatData[i];
  }
}
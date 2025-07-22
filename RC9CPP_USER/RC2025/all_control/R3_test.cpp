#include "R3_test.h"

TaskManager task_core;
dji_motor_handle dji_core;

RC9Protocol esp_port(uart, &huart3), position_port(uart, &huart6);
RC9Protocol ros_port(cdc, nullptr);
RC9Protocol send_port(uart, &huart4);

ros_sensor ros_sensor_;
position position_sensor;
Camera camera;
CameraOperation camera_operation(&camera);
R3_xbox test_xbox(&position_sensor, &ros_sensor_, &camera_operation);

vesc shoot_1(vesc_id_4, &hfdcan1, 7.0f, 1.0f), shoot_2(vesc_id_5, &hfdcan1, 7.0f, 1.0f);
vesc u8_front(vesc_id_1, &hfdcan3), u8_left(vesc_id_2, &hfdcan3), u8_right(vesc_id_3, &hfdcan3);
m3508p m2006_putball(dji_id_1, &hfdcan2, 55.4248f, M2006_MAX_CURRENT, M2006_CURRENT_MAP);
m3508p m3508_left(dji_id_4, &hfdcan2), m3508_front(dji_id_3, &hfdcan2), m3508_right(dji_id_2, &hfdcan2);

RoboChassis s3_chassis(swerve3_chassis);
chassis_info s3_chassis_info = {0.037f, 0.17f, 0.3f, 0.0f, 0.44f, 0.38735f};

AutoYunballR3 auto_yunball;
R3Shooter shooter;
photogate_shoot gate(rising, GPIOD, GPIO_PIN_14);
photogate_shoot gate_down(falling, GPIOE, GPIO_PIN_3);
demo plot;

extern "C"
{
  void r3_setup()
  {
    CanDevice::InitAllFiltersNoMask();
    time_counter::init_time_counter();

    // UART5
    send_port.initQueue();
    send_port.startUartReceiveIT();
    plot.addport(&send_port);
    plot.add_xbox(&test_xbox);

    // LiDAR
    ros_sensor_.add_recolate_imu(&position_sensor);
    ros_sensor_.addport(&ros_port);
    ros_port.initQueue();
    ros_port.startUartReceiveIT();

    // camera
    camera.addport(&ros_port);
    camera_operation.add_chassis(&s3_chassis);

    // position
    position_port.initQueue();
    position_port.startUartReceiveIT();
    position_sensor.addport(&position_port);
    position_sensor.set_map_plot(0.0f, 0.046f);

    // DJI_Motor pid config
    m2006_putball.config_mech_param(55.4248f, 2.0f);
    m2006_putball.rpm_control.config_all(32.0f, 0.76f, 8.6f, 106.0f, 20000.0f, 5.0f);
    m3508_left.rpm_control.config_all(32.0f, 0.76f, 8.6f, 106.0f, 20000.0f, 5.0f);
    m3508_front.rpm_control.config_all(32.0f, 0.76f, 8.6f, 106.0f, 20000.0f, 5.0f);
    m3508_right.rpm_control.config_all(32.0f, 0.76f, 8.6f, 106.0f, 20000.0f, 5.0f);
    m3508_front.config_mech_param(46.71f, 0.0f);
    m3508_front.angle_pid_control.ConfigAll(3.1f, 0.4f, 1.4f, 0.0f, 160.0f, 0.2f, 3.0f);
    m3508_left.config_mech_param(46.71f, 0.0f);
    m3508_left.angle_pid_control.ConfigAll(3.1f, 0.4f, 1.4f, 0.0f, 160.0f, 0.2f, 3.0f);
    m3508_right.config_mech_param(46.71f, 0.0f);
    m3508_right.angle_pid_control.ConfigAll(3.1f, 0.4f, 1.4f, 0.0f, 160.0f, 0.2f, 3.0f);

    // vesc pid config
    shoot_1.rpm_control.config_all(70.0f, 1.0f, 140.0f, 20.0f, 65000, 5.0f);
    shoot_1.add_slave_motor(&shoot_2);
    shoot_1.rpm_control.enable_TD();
    shoot_2.rpm_control.enable_TD();

    // yunball & shooter
    auto_yunball.add_motor(&m2006_putball);
    auto_yunball.add_io(GPIOA, GPIO_PIN_8, GPIOA, GPIO_PIN_2, GPIOG, GPIO_PIN_7, GPIOG, GPIO_PIN_6);

    shooter.init(&shoot_1, &shoot_2);
    shooter.add_gate(&gate, &gate_down);
    gate.set_motors(&shoot_1, &shoot_2);
    gate_down.set_motors(&shoot_1, &shoot_2);

    // Xbox
    esp_port.initQueue();
    esp_port.startUartReceiveIT();
    test_xbox.addport(&esp_port);
    test_xbox.add_R3shooter(&shooter);
    test_xbox.add_chassis(&s3_chassis);
    test_xbox.add_autoyunball(&auto_yunball);

    // chassis
    s3_chassis.add_6_motors(&m3508_front, &u8_front, &m3508_right, &u8_right, &m3508_left, &u8_left);
    s3_chassis.config(s3_chassis_info);
    s3_chassis.pointtrack_config(0.76f, 0.0f, 0.25f, 0.0f, 5.0f, 0.008f, 0.0f);
    s3_chassis.add_imu(&position_sensor);
    s3_chassis.add_photogate(GPIOF, GPIO_PIN_8, GPIOF, GPIO_PIN_9, GPIOD, GPIO_PIN_15, nullptr, 0);
    s3_chassis.yawadjuster_config(0.12f, 0.0f, 0.004f, 0.0f, 5.0f, 0.1f, 0.5f);

    task_core.registerTask(0, &dji_core);
    task_core.registerTask(2, &shoot_1);
    task_core.registerTask(2, &shoot_2);
    task_core.registerTask(3, &u8_front);
    task_core.registerTask(3, &u8_left);
    task_core.registerTask(3, &u8_right);
    task_core.registerTask(3, &shooter);
    task_core.registerTask(4, &s3_chassis);
    task_core.registerTask(5, &ros_port);
    task_core.registerTask(6, &test_xbox);
    task_core.registerTask(7, &auto_yunball);
    task_core.registerTask(8, &position_port);
    task_core.registerTask(3, &plot);
    task_core.registerTask(9, &camera_operation);
    task_core.registerTask(2, &send_port);

    osKernelStart();
  }
}

void demo::process_data()
{
  MX_FDCAN2_Init();
  CanDevice::InitAllFiltersNoMask();

  // lidar TF
//  float yaw = position_sensor.get_heading();
//  if (yaw < 0.0f)
//  {
//    yaw += 360.0f; // 确保航向角在0到360度之间
//  }
//  float arr[7] = {position_sensor.get_world_pos_x(), position_sensor.get_world_pos_y(),
//                  -ros_sensor_.real_radar_world_pos.x, ros_sensor_.real_radar_world_pos.y,
//                  ros_sensor_.ros_radar_loaction.world_pos.x, ros_sensor_.ros_radar_loaction.world_pos.y,
//                  yaw};
//  sendFloatData(1, arr, 7);

     float send_datas[7] = {gate.rpm1,
                           gate.rpm2,
                           shooter.info.auto_rpm,
                           shoot_1.get_rpm(),
                           shoot_2.get_rpm(),
                           (float)shoot_1.target_erpm,
                            (float)shoot_2.target_erpm};

    sendFloatData(1, send_datas, 7); 
}

void demo::DataReceivedCallback(const uint8_t *byteData, const float *floatData, uint8_t id, uint16_t byteCount)
{
  for (int i = 0; i < 4; i++)
  {
    recive_data[i] = floatData[i];
  }

  robot_pos.x = (-floatData[0] + pian_x);
  robot_pos.y = (-floatData[1] + pian_y);
  robot_v.x = floatData[2];
  robot_v.y = floatData[3];
  xbox->robot_point = robot_pos;
}

void demo::add_xbox(R3_xbox *xbox_)
{
  xbox = xbox_;
  pian_x = xbox->center_point.x;
  pian_y = xbox->center_point.y;
}
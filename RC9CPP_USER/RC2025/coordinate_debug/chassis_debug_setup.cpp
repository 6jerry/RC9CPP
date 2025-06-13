#include "chassis_debug_setup.h"

TaskManager task_core;
dji_motor_handle dji_core;
RC9Protocol esp_port(uart, &huart3), position_port(uart, &huart6);
RC9Protocol ros_port(cdc, nullptr);
RC9Protocol Lora_port(uart, &huart4);
RC9Protocol send_port(uart, &huart5);
// Laser laser(&huart3);
ros_sensor ros_sensor_;
position position_sensor;

Encoder encoder(0x01, &hfdcan3);
PolynomialFitter fitter(0.05f, 0.23f, false);

//??
m3508p m2006_left(dji_id_4, &hfdcan1, 111.72384f, M2006_MAX_CURRENT, M2006_CURRENT_MAP), m2006_right(dji_id_1, &hfdcan1, 111.72384f, M2006_MAX_CURRENT, M2006_CURRENT_MAP), m2006_front(dji_id_3, &hfdcan1, 111.72384f, M2006_MAX_CURRENT, M2006_CURRENT_MAP);
m3508p turn_motor(dji_id_2, &hfdcan1, 49.1372f);
vesc u8_front(vesc_id_1, &hfdcan2), u8_left(vesc_id_2, &hfdcan2), u8_right(vesc_id_3, &hfdcan2);
vesc m6374(vesc_id_4, &hfdcan2, 7.0f, 2.0f);
chassis_adjust_xbox chassis_debug(&position_sensor);

RoboChassis s3_chassis(swerve3_chassis);
chassis_info s3_chassis_info = {0.037f, 0.17f, 0.3f, 0.0f, 0.44f, 0.38735f};
AutoShooter auto_shooter;

auto_yunball yunball_port;
demo plot;

extern "C"
{

  void chassis_debug_setup()
  {
    // xbox
    CanDevice::InitAllFiltersNoMask();
    esp_port.initQueue();
    esp_port.startUartReceiveIT();
    // position_sensor.startUartReceiveIT();
    position_port.initQueue();
    position_port.startUartReceiveIT();
    // laser.init();
    // laser.startUartReceiveIT();
    /**************debug*************/
    send_port.initQueue();
    send_port.startUartReceiveIT();
    // s3_xbox.add_sending(&send_port);
    plot.addport(&send_port);
    // plot.add_xbox(&s3_xbox);
    /********************************/
    /****************************************************/
    ros_sensor_.add_recolate_imu(&position_sensor);
    ros_sensor_.addport(&ros_port);
    // ros_port.startUartReceiveIT();
    ros_port.initQueue();
    /****************************************************/

    chassis_debug.addport(&esp_port);
    chassis_debug.add_chassis(&s3_chassis);

    chassis_debug.ros_imu = &ros_sensor_;
    chassis_debug.add_autoyunball(&yunball_port);

    // position
    position_port.initQueue();
    position_port.startUartReceiveIT();
    position_sensor.addport(&position_port);

    // 运球
    yunball_port.add_motor(&turn_motor);
    yunball_port.add_io(GPIOG, GPIO_PIN_6, GPIOG, GPIO_PIN_8, GPIOG, GPIO_PIN_5, GPIOD, GPIO_PIN_14); // 7发射， 8夹爪，6抬升， 5推射
    yunball_port.add_shooter(&auto_shooter);

    //??
    s3_chassis.add_6_motors(&m2006_front, &u8_front, &m2006_right, &u8_right, &m2006_left, &u8_left);
    s3_chassis.config(s3_chassis_info);
    // s3_chassis.yawadjuster_config(0.039f, 0.0f, 0.002f, 0.0f, 5.0f, 0.2f, 0.0f);
    s3_chassis.pointtrack_config(0.76f, 0.0f, 0.25f, 0.0f, 5.0f, 0.008f, 0.0f);
    s3_chassis.add_imu(&position_sensor);
    s3_chassis.add_photogate(GPIOF, GPIO_PIN_8, GPIOF, GPIO_PIN_9, GPIOD, GPIO_PIN_15, nullptr, 0);
    s3_chassis.yawadjuster_config(0.1f, 0.0f, 0.03f, 0.0f, 7.0f, 0.2f, 0.0f);
    turn_motor.rpm_control.config_all(32.0f, 0.76f, 8.6f, 106.0f, 20000.0f, 5.0f);

    // pid config
    m2006_left.rpm_control.config_all(12.0f, 0.9f, 8.6f, 0.0f, 10000.0f, 3.0f);
    m2006_left.angle_pid_control.ConfigAll(3.6f, 0.0f, 6.0f, 0.0f, 120.0f, 0.2f, 3.0f);
    m2006_front.rpm_control.config_all(12.0f, 0.9f, 8.6f, 0.0f, 10000.0f, 3.0f);
    m2006_front.angle_pid_control.ConfigAll(3.6f, 0.0f, 6.0f, 0.0f, 120.0f, 0.2f, 3.0f);
    m2006_right.rpm_control.config_all(12.0f, 0.9f, 8.6f, 0.0f, 10000.0f, 3.0f);
    m2006_right.angle_pid_control.ConfigAll(3.6f, 0.0f, 6.0f, 0.0f, 120.0f, 0.2f, 3.0f);

    // auto_shooter
    auto_shooter.add_encoder(&encoder);
    auto_shooter.add_motor(&m6374);
    auto_shooter.dis_control.ConfigAll(16000.0f, 3.3f, 64.0f, 0.0f, 1800.0f, 0.001f, 0.015f);
    auto_shooter.add_plan_info(400, 400, 1200, 400, 400);
    auto_shooter.add_trigger(GPIOF, GPIO_PIN_5, GPIOG, GPIO_PIN_7);
    auto_shooter.add_fitter(&fitter);
    chassis_debug.add_AutoShooter(&auto_shooter);

    // task register
    task_core.registerTask(0, &dji_core);
    task_core.registerTask(2, &u8_front);
    task_core.registerTask(2, &u8_left);
    task_core.registerTask(2, &u8_right);
    task_core.registerTask(1, &m6374);
    task_core.registerTask(2, &auto_shooter);
    task_core.registerTask(4, &s3_chassis);
    task_core.registerTask(6, &chassis_debug);
    task_core.registerTask(8, &position_port);
    task_core.registerTask(8, &send_port);
    task_core.registerTask(5, &ros_port);
    task_core.registerTask(2, &plot);
    task_core.registerTask(5, &yunball_port);
    osKernelStart();
  }
}

void demo::process_data()
{

  // static Vector2D last_pos = position_sensor.world_pos;
  // float speed_xx  = (position_sensor.world_pos.x - last_pos.x) / 0.04f;
  // float speed_yy  = (position_sensor.world_pos.y - last_pos.y) / 0.04f;
  //  float dis = s3_xbox.get_dis_2_center();
  //  float arr[1] = {dis};
  //  uint8_t instruction[1] = {0};
  //  if(xbox_ptr->shoot_title == 1)
  //  {
  //      instruction[0] = 1;
  //  			xbox_ptr->shoot_title = 0;
  //  			sendByteData(1, instruction, 1);
  //  }
  //  else if(xbox_ptr->yunball_title == 1)
  //  {
  //      instruction[0] = 2;
  //  			xbox_ptr->yunball_title = 0;
  //  			sendByteData(1, instruction, 1);
  //  }
  //  else if(xbox_ptr->just_yun_title == 1)
  //  {
  //  			instruction[0] = 3;
  //  			xbox_ptr->just_yun_title = 0;
  //  			sendByteData(1, instruction, 1);
  //  }
  //  else
  //  {
  //      instruction[0] = 0;
  //  }
  /* ?????1 ???? 2 ???????? 3 position??????????????*/
  /*float arr[6] = {s3_xbox.xbox_msgs.joyLHori_map, s3_xbox.xbox_msgs.joyLVert_map, \
      s3_chassis.target.target_robovel.x, s3_chassis.target.target_robovel.y, \
      speed_xx, speed_yy};
  // ???????
  last_pos = position_sensor.world_pos;
  sendFloatData(1, arr, 6);*/

  //	float arr[6] = {position_sensor.real_world_pos.x, position_sensor.real_world_pos.y, position_sensor.world_pos.x, position_sensor.world_pos.y};

  //  float arr[6] = {position_sensor.get_world_pos_x(), position_sensor.get_world_pos_y(),
  //                  -ros_sensor_.real_radar_world_pos.x, ros_sensor_.real_radar_world_pos.y};

  float arr[4] = {auto_shooter.shoot_info.target_dis, auto_shooter.shoot_info.real_dis, chassis_debug.center_heading, position_sensor.get_heading()};
  sendFloatData(1, arr, 4);
}

void demo::DataReceivedCallback(const uint8_t *byteData, const float *floatData, uint8_t id, uint16_t byteCount)
{
  //    for(int i = 0; i < 4; i++)
  //        {recive_data[i] = floatData[i];}

  //    robot_pos.x = floatData[0];
  //    robot_pos.y = floatData[1];
  //    robot_v.x = floatData[2];
  //    robot_v.y = floatData[3];
}
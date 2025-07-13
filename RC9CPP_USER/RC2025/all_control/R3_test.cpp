#include "R3_test.h"

TaskManager task_core;
dji_motor_handle dji_core;

RC9Protocol esp_port(uart, &huart3), debug_port(uart, &huart6), position_port(uart, &huart4);

vesc shoot_1(vesc_id_1, &hfdcan1, 7.0f, 1.0f);
vesc shoot_2(vesc_id_2, &hfdcan1, 7.0f, 1.0f);

Encoder encoder(0x03, &hfdcan2, 2.0f, 4096.0f);

m3508p front_left_motor(dji_id_3, &hfdcan3), front_right_motor(dji_id_4, &hfdcan3), back_left_motor(dji_id_2, &hfdcan3), back_right_motor(dji_id_1, &hfdcan3);
RoboChassis robot_chassis(omni4_chassis);
chassis_info omni4_info = {0.0719f, 0.0f, 0.0f, 0.2425f, 0.0f, 0.0f};
position position_sensor;
R3_xbox test_xbox(&position_sensor);
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

    position_port.startUartReceiveIT();
    position_port.initQueue();
    front_left_motor.rpm_control.config_all(32.0f, 0.76f, 8.6f, 106.0f, 20000.0f, 5.0f);
    front_right_motor.rpm_control.config_all(32.0f, 0.76f, 8.6f, 106.0f, 20000.0f, 5.0f);
    back_left_motor.rpm_control.config_all(32.0f, 0.76f, 8.6f, 106.0f, 20000.0f, 5.0f);
    back_right_motor.rpm_control.config_all(32.0f, 0.76f, 8.6f, 106.0f, 20000.0f, 5.0f);
    test_xbox.addport(&esp_port);
    test_xbox.gate.add_io_interrupt(GPIOD, GPIO_PIN_14);
    test_xbox.gate_down.add_io_interrupt(GPIOD, GPIO_PIN_15);
    test_xbox.add_chassis(&robot_chassis);
    robot_chassis.add4_motors(&back_right_motor, &front_right_motor, &front_left_motor, &back_left_motor);
    robot_chassis.config(omni4_info);
    robot_chassis.pointtrack_config(0.76f, 0.0f, 0.25f, 0.0f, 5.0f, 0.008f, 0.0f);
    robot_chassis.add_imu(&position_sensor);
    robot_chassis.yawadjuster_config(0.12f, 0.0f, 0.004f, 0.0f, 5.0f, 0.1f, 0.5f);

    shoot_1.rpm_control.config_all(70.0f, 1.0f, 140.0f, 20.0f, 65000, 5.0f);
    shoot_2.rpm_control.config_all(70.0f, 1.0f, 140.0f, 20.0f, 65000, 5.0f);
    shoot_1.rpm_control.enable_TD();
    shoot_2.rpm_control.enable_TD();
    // shoot_1.addport(&debug_port);
    // shoot_1.start_debug();

    position_sensor.set_map_plot(0.0f, 0.36562f);
    position_sensor.addport(&position_port);

    plot.addport(&debug_port);

    test_xbox.init(&shoot_1, &shoot_2, &encoder);
    task_core.registerTask(0, &dji_core);
    task_core.registerTask(2, &shoot_1);
    task_core.registerTask(2, &shoot_2);
    task_core.registerTask(4, &robot_chassis);
    task_core.registerTask(6, &test_xbox);
    task_core.registerTask(3, &debug_port);
    task_core.registerTask(7, &position_port);
    task_core.registerTask(2, &plot);
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

   if(test_flag1 == 1)
  {

    encoder.set_anti_clockwise();

     test_flag1 = 0;
  }
   if(test_flag2 == 1)
  {

    encoder.set_clockwise();

     test_flag2 = 0;
  }
  */
  float send_datas[5] = {test_xbox.gate.rpm1,
                         test_xbox.gate.rpm2,
                         test_xbox.target_rpm,
                         encoder.get_rpm(),
                         encoder.get_distance() * 10000.0f};

  sendFloatData(1, send_datas, 5);
}
#include "chassis_test.h"
#include "Vector2D.h"
#include "ros_sensor.h"
#include "test_laser.h"

TaskManager task_core;
CanManager can_core;
RC9Protocol esp_port(uart, &huart2), position_port(uart, &huart5);
RC9Protocol ros_port(cdc, &huart5);
RC9Protocol Lora_port(uart, &huart4);
RC9Protocol send_port(uart, &huart1);
Laser laser(&huart3);
ros_sensor ros_sensor_;
position position_sensor;

m3508p shooter(2, &hcan1), m3508_left(4, &hcan1, true), m3508_front(3, &hcan1, true), m3508_right(1, &hcan1, true);

chassis_info s3_chassis_info = {0.037f, 0.17f, 0.3f, 0.0f, 0.44f, 0.38735f};

vesc vesc_front(1, &hcan2, 21.0f, 3.0f),
    vesc_left(2, &hcan2, 21.0f, 3.0f), vesc_right(3, &hcan2, 21.0f, 3.0f);

RoboChassis s3_chassis(swerve3_chassis);

auto_lock_test s3_xbox(&position_sensor, &ros_sensor_);

demo plot;

extern "C"
{
    void chassis_move_test(void)
    {
//        can_core.init();
        esp_port.startUartReceiveIT();
        // position_sensor.startUartReceiveIT();
        position_port.initQueue();
        position_port.startUartReceiveIT();
		laser.init();
		laser.startUartReceiveIT();
        /**************debug*************/
        send_port.initQueue();
        send_port.startUartReceiveIT();
        //s3_xbox.add_sending(&send_port);
        plot.addport(&send_port);
		//plot.add_xbox(&s3_xbox);
        /********************************/
        /****************************************************/
        ros_sensor_.add_recolate_imu(&position_sensor);
        ros_sensor_.addport(&ros_port);
        ros_port.startUartReceiveIT();
        /****************************************************/
        m3508_front.config_mech_param(48.26f, 0.0f);
        m3508_front.angle_pid_control.ConfigAll(3.1f, 0.4f, 1.4f, 0.0f, 160.0f, 0.2f, 3.0f);

        m3508_left.config_mech_param(48.26f, 0.0f);
        m3508_left.angle_pid_control.ConfigAll(3.1f, 0.4f, 1.4f, 0.0f, 160.0f, 0.2f, 3.0f);

        m3508_right.config_mech_param(48.26f, 0.0f);
        m3508_right.angle_pid_control.ConfigAll(3.1f, 0.4f, 1.4f, 0.0f, 160.0f, 0.2f, 3.0f);

        s3_chassis.config(s3_chassis_info);
        s3_chassis.add_6_motors(&m3508_front, &vesc_front, &m3508_right, &vesc_right, &m3508_left, &vesc_left);

        s3_chassis.add_photogate(GPIOF, GPIO_PIN_14, GPIOF, GPIO_PIN_15, GPIOG, GPIO_PIN_0, GPIOF, GPIO_PIN_13);

        s3_chassis.enable_debug();

        s3_chassis.add_imu(&position_sensor);
        // s3_chassis.addport(&debug_port);
        s3_chassis.yawadjuster_config(0.039f, 0.0f, 0.002f, 0.0f, 5.0f, 0.2f, 0.0f);
        s3_chassis.pointtrack_config(0.76f, 0.0f, 0.25f, 0.0f, 5.0f, 0.008f, 0.0f);
        s3_chassis.pp_tracker.normal_control.ConfigAll(2.8f, 0.0f, 0.2f, 0.0f, 5.0f, 0.002f, 0.0f);
        s3_chassis.pp_tracker.tangent_control.ConfigAll(1.2f, 0.0f, 3.8f, 0.0f, 1.5f, 0.002f, 0.0f);

        task_core.customize(4, osPriorityRealtime, 10, 20 * 128);
        s3_xbox.addport(&esp_port);
        s3_xbox.add_chassis(&s3_chassis);
        position_sensor.addport(&position_port);
        task_core.registerTask(1, &vesc_front);
        task_core.registerTask(1, &vesc_left);
        task_core.registerTask(1, &vesc_right);

//        task_core.registerTask(0, &can_core);
        task_core.registerTask(2, &s3_chassis);
        task_core.registerTask(9, &s3_xbox);
        task_core.registerTask(8, &position_port);
        task_core.registerTask(8, &send_port);
        task_core.registerTask(9, &plot);
        osKernelStart();
    }
}

void demo::process_data(){

    //static Vector2D last_pos = position_sensor.world_pos;
    //float speed_xx  = (position_sensor.world_pos.x - last_pos.x) / 0.04f;
    //float speed_yy  = (position_sensor.world_pos.y - last_pos.y) / 0.04f;
    // float dis = s3_xbox.get_dis_2_center();
    // float arr[1] = {dis};
	// uint8_t instruction[1] = {0};
    // if(xbox_ptr->shoot_title == 1)
    // {
    //     instruction[0] = 1;
	// 			xbox_ptr->shoot_title = 0;
	// 			sendByteData(1, instruction, 1);
    // }
    // else if(xbox_ptr->yunball_title == 1)
    // {
    //     instruction[0] = 2;
	// 			xbox_ptr->yunball_title = 0;
	// 			sendByteData(1, instruction, 1);
    // }
    // else if(xbox_ptr->just_yun_title == 1)
    // {
	// 			instruction[0] = 3;
	// 			xbox_ptr->just_yun_title = 0;
	// 			sendByteData(1, instruction, 1);
    // }
    // else
    // {
    //     instruction[0] = 0;
    // }
    /* 三组数据：1 手柄速度 2 加速度控制的速度 3 position的速度（姑且认为是真实速度）*/
    /*float arr[6] = {s3_xbox.xbox_msgs.joyLHori_map, s3_xbox.xbox_msgs.joyLVert_map, \
        s3_chassis.target.target_robovel.x, s3_chassis.target.target_robovel.y, \
        speed_xx, speed_yy};
    // 更新上一次速度
    last_pos = position_sensor.world_pos;
    sendFloatData(1, arr, 6);*/

    
}

void demo::DataReceivedCallback(const uint8_t *byteData, const float *floatData, uint8_t id, uint16_t byteCount)
{
    for(int i = 0; i < 4; i++)
        {recive_data[i] = floatData[i];}

    robot_pos.x = floatData[0];
    robot_pos.y = floatData[1];
    robot_v.x = floatData[2];
    robot_v.y = floatData[3];
}
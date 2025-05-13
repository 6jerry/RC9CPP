#include "chassis_test.h"
#include "ros_sensor.h"

TaskManager task_core;
CanManager can_core;
RC9Protocol esp_port(uart, &huart2), position_port(uart, &huart5);
RC9Protocol ros_port(cdc, &huart5);
RC9Protocol send_port(uart, &huart1);
ros_sensor ros_sensor_;
position position_sensor;

m3508p shooter(2, &hcan1), m3508_left(4, &hcan1, true), m3508_front(3, &hcan1, true), m3508_right(1, &hcan1, true);

chassis_info s3_chassis_info = {0.037f, 0.17f, 0.3f, 0.0f, 0.44f, 0.38735f};

vesc vesc_front(1, &hcan2, 21.0f, 3.0f),
    vesc_left(2, &hcan2, 21.0f, 3.0f), vesc_right(3, &hcan2, 21.0f, 3.0f);

RoboChassis s3_chassis(swerve3_chassis);

auto_lock_test s3_xbox(&position_sensor);

demo plot;

extern "C"
{
    void chassis_move_test(void)
    {
        can_core.init();
        esp_port.startUartReceiveIT();
        // position_sensor.startUartReceiveIT();
        position_port.initQueue();
        position_port.startUartReceiveIT();
        /**************debug*************/
        send_port.initQueue();
        plot.addport(&send_port);
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

        task_core.registerTask(0, &can_core);
        task_core.registerTask(2, &s3_chassis);
        task_core.registerTask(2, &s3_xbox);
        task_core.registerTask(8, &position_port);
        task_core.registerTask(8, &send_port);
        task_core.registerTask(9, &plot);
        osKernelStart();
    }
}

void demo::process_data(){
    float arr[7] = { ros_sensor_.real_radar_world_pos.x, ros_sensor_.real_radar_world_pos.y,
	ros_sensor_.ros_radar_loaction.world_pos.x, ros_sensor_.ros_radar_loaction.world_pos.y, 
    position_sensor.get_world_pos().x, position_sensor.get_world_pos().y
	};
    sendFloatData(1, arr, 7);
}
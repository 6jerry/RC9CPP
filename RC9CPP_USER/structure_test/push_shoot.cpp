#include "push_shoot.h"

// 需要调节的参数：

// 左边舵机准备吃球的角度，不是真实角度，你看着调
#define left_up 80

// 左边舵机放下去吃球但又刚好不会把底盘顶起来的角度
#define left_down 138

// 右边舵机准备吃球的角度
#define right_up 183
// 右边舵机放下去吃球但又刚好不会把底盘顶起来的角度
#define right_down 126
// 吃球的坐标
#define target_eat_x 0.04f
#define target_eat_y 1.02f

// 放球的坐标，就是你放球的地方

#define target_push_x -1.36f

#define target_push_y 1.28f

// 吃球的车头朝向，最好对准球的方向
#define target_eat_heading 0.0f
// 初始化延时，就是你放下去之后什么时候开始自动
#define init_delay 120 // 以20ms为单位
// 吃球框放下延时，就是开过去之后什么时候吃球框放下吃球后开往下一点，如果很小就是放下后里马往放置区开，太小就是还没完全放下吃球就开往放置区了
#define eat_delay 66 // 以20ms为单位

#define finish_delay 5 // 别改
// 放球的车头朝向
#define target_push_heading 1.27f

// m3508p m3508_shooter(1, &hcan1), m3508_pitch(2, &hcan1);
// m6020s m6020_test(4, &hcan2);
// vesc vesc_test(1, &hcan1);
TaskManager task_core;
// CanManager can_core;
//  shoot_xbox shoot_control(&m3508_shooter, &m3508_pitch);
RC9Protocol debug(&huart4, false), esp32_serial(&huart2, false);

fdi fdi_test(&huart3);

servo mg996_left(&htim9, TIM_CHANNEL_1), mg996_right(&htim9, TIM_CHANNEL_2);

// swerve4 swerve_test(&vesc_test, &m6020_test);
tb6612 right_back(&htim2, TIM_CHANNEL_1, &htim8, GPIOC, GPIO_PIN_0, GPIOC, GPIO_PIN_1), left_front(&htim2, TIM_CHANNEL_2, &htim4, GPIOC, GPIO_PIN_2, GPIOC, GPIO_PIN_3), right_front(&htim2, TIM_CHANNEL_3, &htim3, GPIOF, GPIO_PIN_1, GPIOF, GPIO_PIN_2), left_back(&htim2, TIM_CHANNEL_4, &htim1, GPIOF, GPIO_PIN_3, GPIOF, GPIO_PIN_4);
odometry odom_test(&right_front, &right_back, &left_back, &left_front, &fdi_test, mecanum4);
mcknum4 mknum_test(&right_front, &right_back, &left_back, &left_front, 0.03f, 0.25f, &fdi_test, &odom_test); // 86.93mm,L 148mm W

shoot_xbox box_test(nullptr, nullptr, &mknum_test);
xbox_r2n resxbox(&mknum_test);
demo test2;

extern "C" void pshoot_setup(void)
{
    // box_test.rcninit(2);
    // esp32_serial.rcninit(1);

    // esp32_serial.msgbuff_pub.init("xboxbuff", SYN, &esp32_serial);
    // box_test.buff_sub.init("xboxbuff", SYN, &box_test);
    // can_core.init();
    right_front.init();
    right_back.init();
    left_front.init();
    left_back.init();
    mg996_left.init();
    mg996_right.init();

    debug.startUartReceiveIT();
    esp32_serial.startUartReceiveIT();
    // wwwwfdi_test.startUartReceiveIT();
    esp32_serial.addsubscriber(&resxbox);
    // task_core.registerTask(0, &can_core);
    // task_core.registerTask(1, &vesc_test);
    // task_core.registerTask(1, &box_test);
    task_core.registerTask(2, &mknum_test);
    task_core.registerTask(4, &right_front);
    task_core.registerTask(4, &right_back);
    task_core.registerTask(5, &left_front);
    task_core.registerTask(5, &left_back);
    task_core.registerTask(6, &odom_test);
    task_core.registerTask(6, &resxbox);
    task_core.registerTask(8, &debug);
    task_core.registerTask(7, &test2);
    //  task_core.registerTask(7, &esp32_serial);

    debug.tx_frame_mat.frame_id = 1;
    debug.tx_frame_mat.data_length = 24;

    mg996_left.set_ccr(left_up); // 146 最低位,80最高位
    mg996_right.set_ccr(right_up);
    resxbox.rcninit(2);
    esp32_serial.rcninit(1);
    debug.rcninit(3);
    debug.portinit(7);
    // mg996_left.set_ccr(150);
    resxbox.SERVO = &mg996_left;
    resxbox.servo_right = &mg996_right;

    // mknum_test.heading_pid.error > -0.01f;
    // mknum_test.heading_pid.error < 0.01f;
    osKernelStart();
}

void demo::process_data()
{
    // vesc_test.rpm_control.PID_SetParameters(debug.rx_frame_mat.data.msg_get[0], debug.rx_frame_mat.data.msg_get[1], debug.rx_frame_mat.data.msg_get[2]);
    // mknum_test.pointracker.track_pid.PID_SetParameters(debug.rx_frame_mat.data.msg_get[0], debug.rx_frame_mat.data.msg_get[1], debug.rx_frame_mat.data.msg_get[2]);
    // right_front.rpm_control.increPID_SetParameters(debug.rx_frame_mat.data.msg_get[0], debug.rx_frame_mat.data.msg_get[1], debug.rx_frame_mat.data.msg_get[2], debug.rx_frame_mat.data.msg_get[3]);

    // debug.tx_frame_mat.data.msg_get[0] = odom_test.now_heading;

    ppsend_Asyn(7, 9, 0, &mknum_test.target_w);
    //  debug.tx_frame_mat.data.msg_get[2] = right_front.rpm_control.setpoint / 45.0f;
    //    debug.tx_frame_mat.data.msg_get[2] = vesc_test.target_rpm;
    //     debug.tx_frame_mat.data.msg_get[2] = m3508_shooter.rpm_control.setpoint;
    //      ppget_AsynOverwrite();
    //      testdd += 0.001f;

    // m6020_test.pos_pid.PID_SetParameters(debug.rx_frame_mat.data.msg_get[0], debug.rx_frame_mat.data.msg_get[1], debug.rx_frame_mat.data.msg_get[2]);

    // debug.tx_frame_mat.data.msg_get[0] = m6020_test.get_rpm();

    // debug.tx_frame_mat.data.msg_get[1] = m6020_test.target_angle;
    // debug.tx_frame_mat.data.msg_get[2] = m6020_test.angle_error;
}

uint8_t demo::msgin(uint8_t rcnID_, const void *data)
{
    const float *inputData = static_cast<const float *>(data);
    if (inputData)
    {
        testdata[0] = inputData[0];
        testdata[1] = inputData[1];
        testdata[2] = inputData[2];
        return 1;
    }
    return 0;
}
uint8_t demo::msgout(uint8_t rcnID_, void *output)
{
    return 0;
}
demo::demo(float init_) : testdd(init_)
{
}
#include "push_shoot.h"

m3508p m3508_shooter(1, &hcan1), m3508_pitch(2, &hcan1);
m6020s m6020_test(4, &hcan2);
vesc vesc_test(1, &hcan1);
TaskManager task_core;
CanManager can_core;
// shoot_xbox shoot_control(&m3508_shooter, &m3508_pitch);
RC9Protocol debug(&huart5, false), esp32_serial(&huart2, false);

fdi fdi_test(&huart4);

servo mg996_left(&htim9, TIM_CHANNEL_1);

swerve4 swerve_test(&vesc_test, &m6020_test);
tb6612 right_back(&htim2, TIM_CHANNEL_1, &htim8, GPIOC, GPIO_PIN_0, GPIOC, GPIO_PIN_1), left_front(&htim2, TIM_CHANNEL_2, &htim4, GPIOC, GPIO_PIN_2, GPIOC, GPIO_PIN_3), right_front(&htim2, TIM_CHANNEL_3, &htim3, GPIOF, GPIO_PIN_1, GPIOF, GPIO_PIN_2), left_back(&htim2, TIM_CHANNEL_4, &htim1, GPIOF, GPIO_PIN_3, GPIOF, GPIO_PIN_4);
odometry odom_test(&left_front, &right_front, &left_back, &right_back, &fdi_test, mecanum4);
mcknum4 mknum_test(&right_front, &right_back, &left_back, &left_front, 0.03f, 0.25f, &fdi_test, &odom_test);//86.93mm,L 148mm W

shoot_xbox box_test(&m3508_pitch, &m3508_shooter, &mknum_test);

demo test2;

extern "C" void pshoot_setup(void)
{
    // box_test.rcninit(2);
    // esp32_serial.rcninit(1);

    // esp32_serial.msgbuff_pub.init("xboxbuff", SYN, &esp32_serial);
    // box_test.buff_sub.init("xboxbuff", SYN, &box_test);
    can_core.init();
    right_front.init();
    right_back.init();
    left_front.init();
    left_back.init();
    mg996_left.init();

    debug.startUartReceiveIT();
    esp32_serial.startUartReceiveIT();
    fdi_test.startUartReceiveIT();
    esp32_serial.addsubscriber(&box_test);
    // task_core.registerTask(0, &can_core);
    // task_core.registerTask(1, &vesc_test);
    task_core.registerTask(1, &box_test);
    task_core.registerTask(2, &mknum_test);
    task_core.registerTask(4, &right_front);
    task_core.registerTask(4, &right_back);
    task_core.registerTask(5, &left_front);
    task_core.registerTask(5, &left_back);
    task_core.registerTask(6, &odom_test);
    task_core.registerTask(8, &debug);
    task_core.registerTask(7, &test2);
    // task_core.registerTask(7, &esp32_serial);

    debug.tx_frame_mat.frame_id = 1;
    debug.tx_frame_mat.data_length = 24;

    mg996_left.set_ccr(146);//146 最低位,80最高位
    //mg996_left.set_ccr(150);

    osKernelStart();
}

void demo::process_data()
{
    // vesc_test.rpm_control.PID_SetParameters(debug.rx_frame_mat.data.msg_get[0], debug.rx_frame_mat.data.msg_get[1], debug.rx_frame_mat.data.msg_get[2]);

    // right_front.rpm_control.increPID_SetParameters(debug.rx_frame_mat.data.msg_get[0], debug.rx_frame_mat.data.msg_get[1], debug.rx_frame_mat.data.msg_get[2], debug.rx_frame_mat.data.msg_get[3]);

    debug.tx_frame_mat.data.msg_get[0] = right_front.get_rpm();

    debug.tx_frame_mat.data.msg_get[1] = right_front.target_rpm;
    debug.tx_frame_mat.data.msg_get[2] = right_front.rpm_control.setpoint / 45.0f;
    //  debug.tx_frame_mat.data.msg_get[2] = vesc_test.target_rpm;
    //   debug.tx_frame_mat.data.msg_get[2] = m3508_shooter.rpm_control.setpoint;
    //    ppget_AsynOverwrite();
    //    testdd += 0.001f;

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
#include "r2n_setup.h"
RC9Protocol esp32_serial(&huart1, false),
    data_chain(&huart5, false);
m3508p m3508_front(1, &hcan1), m3508_left(3, &hcan1), m3508_right(2, &hcan1); // 九期r2，硬件连接：三只m3508作为底盘动力电机位于can1
TaskManager task_core;
CanManager can_core;
action Action(&huart3, -160.0f, 120.0f, true);
omni3 r2n_chassis(&m3508_front, &m3508_right, &m3508_left, 0.0719f, 0.406f, &Action, 4.0f, 0.0f, 1.0f, 0.0086f, 0.0f, 0.026f);
xbox_r2n r2_remote(&Action, &r2n_chassis);
demo test2, test3;
// go1can go1(0, &hcan1, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f); // 0.017f, 0.000024f, 0.00001f

extern "C" void
r2n_setup(void)
{
    can_core.init();

    // r2_remote.GO1 = &go1;
    Action.startUartReceiveIT();
    esp32_serial.startUartReceiveIT();

    data_chain.startUartReceiveIT();
    esp32_serial.addsubscriber(&r2_remote);

    task_core.registerTask(0, &can_core);
    // task_core.registerTask(4, &go1);
    task_core.registerTask(3, &r2n_chassis);
    task_core.registerTask(2, &r2_remote);
    task_core.registerTask(8, &test2);

    // 以上为首次创建的新任务

    task_core.registerTask(8, &data_chain);
    r2_remote.m3 = &m3508_front;
    data_chain.tx_frame_mat.data_length = 24;
    data_chain.tx_frame_mat.frame_id = 1;

    osKernelStart();
}

void demo::process_data()
{
    /*data_chain.tx_frame_mat.data.msg_get[0] = go1.real_speed;
    data_chain.tx_frame_mat.data.msg_get[1] = go1.speed_ladrc.V1;
    data_chain.tx_frame_mat.data.msg_get[2] = go1.speed_ladrc.z1;
    data_chain.tx_frame_mat.data.msg_get[3] = go1.speed_ladrc.z3;
    data_chain.tx_frame_mat.data.msg_get[4] = go1.speed_ladrc.u;

    go1.speed_ladrc.ladrc_SetParameters(data_chain.rx_frame_mat.data.msg_get[0], data_chain.rx_frame_mat.data.msg_get[1], data_chain.rx_frame_mat.data.msg_get[2], data_chain.rx_frame_mat.data.msg_get[3], data_chain.rx_frame_mat.data.msg_get[4], data_chain.rx_frame_mat.data.msg_get[5]);*/
    m3508_front.rpm_control.increPID_SetParameters(data_chain.rx_frame_mat.data.msg_get[0], data_chain.rx_frame_mat.data.msg_get[1], data_chain.rx_frame_mat.data.msg_get[2], data_chain.rx_frame_mat.data.msg_get[3]);

    // m3508_front.set_rpm(data_chain.rx_frame_mat.data.msg_get[5]);
    data_chain.tx_frame_mat.data.msg_get[0] = m3508_front.get_rpm();

    data_chain.tx_frame_mat.data.msg_get[1] = m3508_front.target_rpm;
    data_chain.tx_frame_mat.data.msg_get[2] = m3508_front.rpm_control.setpoint / M3508_G;
    data_chain.tx_frame_mat.data.msg_get[3] = m3508_front.rpm_control.output / 10.0f;
}
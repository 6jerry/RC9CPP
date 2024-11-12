#include "r2n_setup.h"
RC9Protocol esp32_serial(&huart1, false),
    data_chain(&huart5, false);
m3508p m3508_front(1, &hcan1), m3508_left(3, &hcan1), m3508_right(2, &hcan1); // 九期r2，硬件连接：三只m3508作为底盘动力电机位于can1
TaskManager task_core;
CanManager can_core;
action Action(&huart3, -160.0f, 120.0f, true);
omni3 r2n_chassis(&m3508_front, &m3508_right, &m3508_left, 0.0719f, 0.406f, &Action, 7.0f, 0.0f, 0.7f, 0.0086f, 0.0f, 0.026f);
xbox_r2n r2_remote(&Action, &r2n_chassis);
demo test2, test3;
go1can go1(0, &hcan1, 0.0f, 0.0f, 0.0f);

extern "C" void
r2n_setup(void)
{
    can_core.init();

    r2_remote.GO1 = &go1;
    Action.startUartReceiveIT();
    esp32_serial.startUartReceiveIT();

    data_chain.startUartReceiveIT();
    esp32_serial.addsubscriber(&r2_remote);

    task_core.registerTask(4, &can_core);
    task_core.registerTask(1, &go1);
    task_core.registerTask(3, &r2n_chassis);
    task_core.registerTask(2, &r2_remote);
    task_core.registerTask(8, &test2);

    // 以上为首次创建的新任务

    task_core.registerTask(8, &data_chain);

    data_chain.tx_frame_mat.data_length = 24;
    data_chain.tx_frame_mat.frame_id = 1;
    osKernelStart();
}
void demo::process_data()
{
    data_chain.tx_frame_mat.data.msg_get[0] = go1.real_speed;
    data_chain.tx_frame_mat.data.msg_get[1] = go1.target_rpm;
    data_chain.tx_frame_mat.data.msg_get[2] = go1.real_t;

    go1.rpm_pid.PID_SetParameters(data_chain.rx_frame_mat.data.msg_get[0], data_chain.rx_frame_mat.data.msg_get[1], data_chain.rx_frame_mat.data.msg_get[2]);
}
#include "r1n_setup.h"

RC9Protocol esp32_serial(&huart1, false),
    data_chain(&huart5, false);
m3508p m3508_right(1, &hcan1, 19, 16.7f, 0.21f, 2.4f), m3508_front(3, &hcan1, 19, 16.5f, 0.18f, 2.6f), m3508_left(2, &hcan1, 19, 16.2f, 0.17f, 2.87f);

TaskManager task_core;
CanManager can_core;
action Action(&huart3, 225.2f, 179.54f, false);
omni3_unusual r1n_chassis(&m3508_front, &m3508_right, &m3508_left, 0.0719f, &Action, 7.0f, 0.0f, 0.7f);
xbox_r2n r2_remote(&Action, &r1n_chassis);
demo test2, test3;
// go1can go1(0, &hcan1, 0.017f, 0.000024f, 0.00001f);

extern "C" void
r1n_setup(void)
{
    can_core.init();

    Action.startUartReceiveIT();
    esp32_serial.startUartReceiveIT();

    data_chain.startUartReceiveIT();
    esp32_serial.addsubscriber(&r2_remote);

    task_core.registerTask(0, &can_core);
    // task_core.registerTask(3, &go1);
    task_core.registerTask(2, &r1n_chassis);
    task_core.registerTask(2, &r2_remote);
    task_core.registerTask(5, &test2);

    // 以上为首次创建的新任务

    task_core.registerTask(4, &data_chain);

    data_chain.tx_frame_mat.data_length = 24;
    data_chain.tx_frame_mat.frame_id = 1;

    osKernelStart();
}
void demo::process_data()
{
    data_chain.tx_frame_mat.data.msg_get[0] = Action.pose_data.world_pos_x;
    data_chain.tx_frame_mat.data.msg_get[1] = Action.pose_data.world_pos_y;

    r1n_chassis.point_track_info.target_x = data_chain.rx_frame_mat.data.msg_get[0];
    r1n_chassis.point_track_info.target_y = data_chain.rx_frame_mat.data.msg_get[1];
    
}
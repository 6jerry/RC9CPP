#include "LiGe_setup.h"
RC9Protocol esp32_serial(&huart5, false), data_chain(&huart1, false);
go1 shooter(&huart2);
TaskManager task_core;

extern "C" void lige_setup()
{
    esp32_serial.startUartReceiveIT();
    shooter.startUartReceiveIT();
    data_chain.startUartReceiveIT();
    data_chain.tx_frame_mat.data_length = 24;
    data_chain.tx_frame_mat.frame_id = 1;
    data_chain.tx_frame_mat.data.msg_get[0] = 1.2;
    data_chain.tx_frame_mat.data.msg_get[1] = 2.2;
    data_chain.tx_frame_mat.data.msg_get[2] = 3.2;
    data_chain.tx_frame_mat.data.msg_get[3] = 4.2;
    data_chain.tx_frame_mat.data.msg_get[4] = 5.2;
    data_chain.tx_frame_mat.data.msg_get[5] = 6.2;

    task_core.registerTask(2, &shooter);
    task_core.registerTask(8, &data_chain);
    osKernelStart();
}
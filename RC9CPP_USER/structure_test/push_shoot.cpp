#include "push_shoot.h"

m3508p m3508_shooter(1, &hcan1), m3508_pitch(2, &hcan1);
// vesc vesc_test(1, &hcan2);
TaskManager task_core;
CanManager can_core;
shoot_xbox shoot_control(&m3508_shooter, &m3508_pitch);
RC9Protocol debug(&huart5, false), esp32_serial(&huart1, false);

demo test2;
extern "C" void pshoot_setup(void)
{
    can_core.init();

    debug.startUartReceiveIT();
    esp32_serial.startUartReceiveIT();
    esp32_serial.addsubscriber(&shoot_control);
    task_core.registerTask(0, &can_core);
    // task_core.registerTask(1, &vesc_test);
    task_core.registerTask(1, &shoot_control);
    task_core.registerTask(8, &debug);
    task_core.registerTask(8, &test2);

    debug.tx_frame_mat.frame_id = 1;
    debug.tx_frame_mat.data_length = 24;

    osKernelStart();
}

void demo::process_data()
{
    /*m3508_shooter.rpm_control.increPID_SetParameters(debug.rx_frame_mat.data.msg_get[0], debug.rx_frame_mat.data.msg_get[1], debug.rx_frame_mat.data.msg_get[2], debug.rx_frame_mat.data.msg_get[3]);

    debug.tx_frame_mat.data.msg_get[0] = m3508_shooter.get_rpm();

    debug.tx_frame_mat.data.msg_get[1] = m3508_shooter.target_rpm;

    debug.tx_frame_mat.data.msg_get[2] = m3508_shooter.rpm_control.setpoint;*/
}
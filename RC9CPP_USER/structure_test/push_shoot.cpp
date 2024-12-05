#include "push_shoot.h"

m3508p m3508_shooter(1, &hcan1), m3508_pitch(2, &hcan1);
// vesc vesc_test(1, &hcan2);
TaskManager task_core;
CanManager can_core;
// shoot_xbox shoot_control(&m3508_shooter, &m3508_pitch);
RC9Protocol debug(&huart5, false), esp32_serial(&huart1, false);

shoot_xbox box_test(&m3508_shooter, &m3508_pitch);


extern "C" void pshoot_setup(void)
{
    box_test.rcninit(2);
    esp32_serial.rcninit(1);

    esp32_serial.msgbuff_pub.init("xboxbuff", SYN, &esp32_serial);
    box_test.buff_sub.init("xboxbuff", SYN, &box_test);
    can_core.init();

    debug.startUartReceiveIT();
    esp32_serial.startUartReceiveIT();
    // esp32_serial.addsubscriber(&shoot_control);
    task_core.registerTask(0, &can_core);
    // task_core.registerTask(1, &vesc_test);
    task_core.registerTask(1, &box_test);
    task_core.registerTask(8, &debug);
    task_core.registerTask(7, &esp32_serial);

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
    ppget_AsynOverwrite();
    testdd += 0.001f;
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
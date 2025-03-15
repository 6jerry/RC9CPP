#include "YunBall_test.h"

TaskManager task_core;
CanManager can_core;
RC9Protocol esp_port(uart, &huart2), debug_port(uart, &huart5);

LaserProcessor laser(&huart6);

m3508p shooter(3, &hcan2), pitcher(1, &hcan2, true), lifter(2, &hcan2, true), turnner(4, &hcan2); // 抬升电机，俯仰电机

moters_debug_xbox m3508_debuger;

// m6020s m6020_front(3, &hcan1), m6020_left(1, &hcan1), m6020_right(2, &hcan1); // 舵向电机
vesc vesc1(1, &hcan1), vesc2(2, &hcan1), vesc3(3, &hcan1), m8080(4, &hcan1, 7.0f, 1.0f);
yun_ball_xbox xbox_test;
extern "C"
{
    void yunball_test_setup(void)
    {
        can_core.init();
        esp_port.startUartReceiveIT();
        laser.startUartReceiveIT();
        debug_port.initQueue();
        pitcher.start_debug();
        pitcher.addport(&debug_port);
        pitcher.config_mech_param(19.2032f, 35.0f); // 光电门f6
        xbox_test.addport(&esp_port);
        xbox_test.add_motor(&pitcher, &lifter, &m8080, &turnner);
        xbox_test.add_trigger(GPIOC, GPIO_PIN_15, GPIOC, GPIO_PIN_13); // shooter c 13
        task_core.registerTask(0, &can_core);
        task_core.registerTask(3, &xbox_test);
        task_core.registerTask(2, &m8080);
        // task_core.registerTask(2, &vesc2);
        // task_core.registerTask(2, &vesc3);
        task_core.registerTask(8, &debug_port);

        osKernelStart();
    }

    void m3508_adjust(void)
    {
        can_core.init();
        esp_port.startUartReceiveIT();
        debug_port.initQueue();
        pitcher.start_debug();
        pitcher.addport(&debug_port);

        pitcher.distance_pid_control.ConfigAll(4.0f, 0.0f, 0.086f, 0.0f, 430.0f, 1.0f, 0.0f);

        m3508_debuger.addport(&esp_port);
        m3508_debuger.add_motor(&lifter);
        pitcher.config_mech_param(19.2032f, 35.0f);

        lifter.config_mech_param(49.1376f, 0.0f);
        lifter.angle_pid_control.ConfigAll(0.0f, 0.0f, 0.0f, 0.0f, 160.0f, 0.2f, 0.0f);
        task_core.registerTask(0, &can_core);
        task_core.registerTask(3, &m3508_debuger);
        task_core.registerTask(8, &debug_port);

        osKernelStart();
    }
    void SendInitCommands()
    {
        for (int i = 0; i < LaserProcessor::CMD_GROUP_SIZE; i++)
        {
            int retry = 0;
            do
            {
                // 发送命令
                const auto &cmd = laser.InitCommands()[i];
                HAL_UART_Transmit(&huart6, cmd.data, cmd.length, 100);

                // 重置状态
                laser.cmd_tracker_[i].sent_time = HAL_GetTick();
                laser.cmd_tracker_[i].status = LaserProcessor::CMD_PENDING;

                // 等待响应（200ms超时）
                while ((HAL_GetTick() - laser.cmd_tracker_[i].sent_time) < 200)
                {
                    if (laser.GetCmdStatus(i) != LaserProcessor::CMD_PENDING)
                        break;
                    HAL_Delay(10);
                }

                // 处理超时
                if (laser.GetCmdStatus(i) == LaserProcessor::CMD_PENDING)
                {
                    laser.cmd_tracker_[i].status = LaserProcessor::CMD_TIMEOUT;
                }

            } while (retry++ < 3 &&
                     (laser.GetCmdStatus(i) == LaserProcessor::CMD_TIMEOUT ||
                      laser.GetCmdStatus(i) == LaserProcessor::CMD_CHECKSUM_ERR));

            HAL_Delay(50); // 保持协议要求的时间间隔
        }
    }
}

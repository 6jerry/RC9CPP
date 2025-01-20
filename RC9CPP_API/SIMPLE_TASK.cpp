#include "SIMPLE_TASK.h"

/*
RC9Protocol esp32_serial(&huart1, false), data_chain(&huart2, false); // 九期的通用协议类，可用于ros调参，ros通讯,与esp32通讯等
action Action(&huart3, 225.2f, 179.54f, false);
TaskManager task_core; // 在这里初始化任务核心不会创建任何任务，只有后面首次注册指定的实例到指定的任务中才会创建任务
CanManager can_core;
m3508p m3508_right(1, &hcan1, 19, 16.7f, 0.21f, 2.4f), m3508_front(3, &hcan1, 19, 16.5f, 0.18f, 2.6f), m3508_left(2, &hcan1, 19, 16.2f, 0.17f, 2.87f), m3508_shoot1(1, &hcan2); // 九期r1，硬件连接，三只m3508-19底盘动力电机位于can1，四只m3508-1作为机构动力电机位于can2
// 以上是基本模块，相当于乐高积木中的最小零件
// 以下为较大的模块，由基本模块拼接而成，开始拼乐高制作整车！你只需了解一些模块的基本拼接规则即可！！
// 库中提供一些常用的底盘，其中omni3_unusual是极为特殊的一款，只能九期r1用！
omni3_unusual r1_chassis(&m3508_front, &m3508_right, &m3508_left, 0.0719f, &Action, 7.0f, 0.0f, 0.7f); // 组装r1的底盘，只需简单将底盘和数个动力电机和一个定位模块拼接在一起，这里的动力电机可不止3508哦，只要派生自动力电机的接口就行，特别的，有些底盘比如说舵轮底盘还需要同时将动力电机和伺服电机组装起来，在库看来，所有电机都分为两类：动力电机和伺服电机，位置控制的3508属于伺服电机
// 组装r1的远程遥控器
xbox_r1n r1_remote(&Action, &r1_chassis);
demo test1;
*/

/*
Vector2D p0(0.0f, 0.0f), p1(-20.44f, 155.29f), p2(-80.38f, 300.0f), p3(-175.74f, 424.26f),
    p4(-300.0f, 519.62f), p5(-444.71f, 579.56f), p6(-600.0f, 600.0f), p7(-755.29f, 579.56f),
    p8(-900.0f, 519.62f), p9(-1024.26f, 424.26f), p10(-1119.62f, 300.0f), p11(-1179.56f, 155.29f),
    p12(-1200.0f, 0.0f), p13(-1179.56f, -155.29f), p14(-1119.62f, -300.0f), p15(-1024.26f, -424.26f),
    p16(-900.0f, -519.62f), p17(-755.29f, -579.56f), p18(-600.0f, -600.0f), p19(-444.71f, -579.56f),
    p20(-300.0f, -519.62f), p21(-175.74f, -424.26f), p22(-80.38f, -300.0f), p23(-20.44f, -155.29f), p24(0.0f, 0.0f);
    */

Vector2D p0(0.0f, 0.0f), p1(54.52f, 414.11f), p2(214.36f, 800.00f), p3(468.63f, 1131.37f),
    p4(800.00f, 1385.64f), p5(1185.89f, 1545.48f), p6(1600.00f, 1600.00f), p7(2014.11f, 1545.48f),
    p8(2400.00f, 1385.64f), p9(2731.37f, 1131.37f), p10(2985.64f, 800.00f), p11(3145.48f, 414.11f),
    p12(3200.00f, 0.0f), p13(3254.52f, -414.11f), p14(3414.36f, -800.00f), p15(3668.63f, -1131.37f),
    p16(4000.00f, -1385.64f), p17(4385.89f, -1545.48f), p18(4800.00f, -1600.00f), p19(5214.11f, -1545.48f),
    p20(5600.00f, -1385.64f), p21(5931.37f, -1131.37f), p22(6185.64f, -800.00f), p23(6345.48f, -414.11f),
    p24(6400.00f, 0.0f),
    p25(6345.48f, 414.11f), p26(6185.64f, 800.00f), p27(5931.37f, 1131.37f),
    p28(5600.00f, 1385.64f), p29(5214.11f, 1545.48f), p30(4800.00f, 1600.00f), p31(4385.89f, 1545.48f),
    p32(4000.00f, 1385.64f), p33(3668.63f, 1131.37f), p34(3414.36f, 800.00f), p35(3254.52f, 414.11f),
    p36(3200.00f, 0.0f), p37(3145.48f, -414.11f), p38(2985.64f, -800.00f), p39(2731.37f, -1131.37f),
    p40(2400.00f, -1385.64f), p41(2014.11f, -1545.48f), p42(1600.00f, -1600.00f), p43(1185.89f, -1545.48f),
    p44(800.00f, -1385.64f), p45(468.63f, -1131.37f), p46(214.36f, -800.00f), p47(54.52f, -414.11f),
    p48(0.0f, 0.0f);

bool change_flag = true;
Vector2D sum;
float dot_sum = 0.0f;
uint8_t tracking_point = 1;

RC9Protocol esp32_serial(&huart1, false),
    data_chain(&huart5, true);
m3508p m3508_front(1, &hcan1), m3508_left(3, &hcan1), m3508_right(2, &hcan1); // 九期r2，硬件连接：三只m3508作为底盘动力电机位于can1
TaskManager task_core;
CanManager can_core;
action Action(&huart3, -160.0f, 120.0f, true);
omni3 r2n_chassis(&m3508_front, &m3508_right, &m3508_left, 0.0719f, 0.406f, &Action, 7.0f, 0.0f, 0.7f, 0.0086f, 0.0f, 0.026f);
xbox_r2n r2_remote(&Action, &data_chain, &r2n_chassis);
demo test1;

/*
RC9Protocol esp32_serial(&huart1, false);
RC9Protocol ros_serial(&huart2, false);
m3508p m3508_right_front(2, &hcan1),
    m3508_right_back(1, &hcan1), m3508_left_front(3, &hcan1), m3508_left_back(4, &hcan1);
// m6020s m6020_shoot(1, &hcan2, false, 120.0f, 0.064f, 5.0f, 0.0f, 0.0f, 0.0f);
action Action(&huart3, 0, 0, false);
TaskManager task_core;
CanManager can_core;

omni4 r1e_chassis(&m3508_right_front, &m3508_right_back, &m3508_left_back, &m3508_left_front, 0.076f, 0.40f, &Action, 6.0f, 0.0f, 0.7f, 0.0f, 0.0f, 0.0f);
xbox_r1n r1_remote(&Action, &r1e_chassis);
demo test1;
*/

/*八期r2
RC9Protocol ros_serial(&huart1, false);
m6020s m6020_back_right(1, &hcan1, 270.0f, 1.8f, 6.0f, 6.0f, 0.0f, 2.0f), m6020_back_left(4, &hcan1, 275.0f, 1.8f, 5.6f, 5.0f, 0.0f, 2.0f), m6020_front_right(2, &hcan1, 272.5f, 1.8f, 5.8f, 5.0f, 0.0f, 2.0f), m6020_front_left(3, &hcan1, 272.5f, 1.8f, 5.8f, 5.0f, 0.0f, 2.0f);

TaskManager task_core;
CanManager can_core;
demo test1;
*/

extern "C" void create_tasks(void)
{

    /*
        esp32_serial.startUartReceiveIT();
        esp32_serial.addsubscriber(&r1_remote);
        data_chain.startUartReceiveIT();
        Action.startUartReceiveIT();
        can_core.init();
        // task_core.registerTask(4, &ros_serial); // 首次将ros_serial注册到任务4，创建任务4
        task_core.registerTask(0, &can_core);
        task_core.registerTask(3, &r1_chassis); // 又将另一个实例注册到了任务4，不会重复创建任务4
        task_core.registerTask(2, &r1_remote);
        task_core.registerTask(4, &data_chain);
        task_core.registerTask(3, &test1);

        data_chain.tx_frame_mat.data_length = 8;
        data_chain.tx_frame_mat.frame_id = 1;
        */

    can_core.init();
    Action.startUartReceiveIT();
    esp32_serial.startUartReceiveIT();

    data_chain.startUartReceiveIT();
    esp32_serial.addsubscriber(&r2_remote);

    task_core.registerTask(0, &can_core);
    task_core.registerTask(3, &r2n_chassis);
    task_core.registerTask(2, &r2_remote);

    task_core.registerTask(3, &test1);

    /*
        r2n_chassis.pure_pursuit_info.path[0] = p0;
        r2n_chassis.pure_pursuit_info.path[1] = p1;
        r2n_chassis.pure_pursuit_info.path[2] = p2;
        r2n_chassis.pure_pursuit_info.path[3] = p3;
        r2n_chassis.pure_pursuit_info.path[4] = p4;
        r2n_chassis.pure_pursuit_info.path[5] = p5;
        r2n_chassis.pure_pursuit_info.path[6] = p6;
        r2n_chassis.pure_pursuit_info.path[7] = p7;
        r2n_chassis.pure_pursuit_info.path[8] = p8;
        r2n_chassis.pure_pursuit_info.path[9] = p9;
        r2n_chassis.pure_pursuit_info.path[10] = p10;
        r2n_chassis.pure_pursuit_info.path[11] = p11;
        r2n_chassis.pure_pursuit_info.path[12] = p12;
        r2n_chassis.pure_pursuit_info.path[13] = p13;
        r2n_chassis.pure_pursuit_info.path[14] = p14;
        r2n_chassis.pure_pursuit_info.path[15] = p15;
        r2n_chassis.pure_pursuit_info.path[16] = p16;
        r2n_chassis.pure_pursuit_info.path[17] = p17;
        r2n_chassis.pure_pursuit_info.path[18] = p18;
        r2n_chassis.pure_pursuit_info.path[19] = p19;
        r2n_chassis.pure_pursuit_info.path[20] = p20;
        r2n_chassis.pure_pursuit_info.path[21] = p21;
        r2n_chassis.pure_pursuit_info.path[22] = p22;
        r2n_chassis.pure_pursuit_info.path[23] = p23;
        r2n_chassis.pure_pursuit_info.path[24] = p24;
        */

    r2n_chassis.pure_pursuit_info.path[0] = p0;
    r2n_chassis.pure_pursuit_info.path[1] = p1;
    r2n_chassis.pure_pursuit_info.path[2] = p2;
    r2n_chassis.pure_pursuit_info.path[3] = p3;
    r2n_chassis.pure_pursuit_info.path[4] = p4;
    r2n_chassis.pure_pursuit_info.path[5] = p5;
    r2n_chassis.pure_pursuit_info.path[6] = p6;
    r2n_chassis.pure_pursuit_info.path[7] = p7;
    r2n_chassis.pure_pursuit_info.path[8] = p8;
    r2n_chassis.pure_pursuit_info.path[9] = p9;
    r2n_chassis.pure_pursuit_info.path[10] = p10;
    r2n_chassis.pure_pursuit_info.path[11] = p11;
    r2n_chassis.pure_pursuit_info.path[12] = p12;
    r2n_chassis.pure_pursuit_info.path[13] = p13;
    r2n_chassis.pure_pursuit_info.path[14] = p14;
    r2n_chassis.pure_pursuit_info.path[15] = p15;
    r2n_chassis.pure_pursuit_info.path[16] = p16;
    r2n_chassis.pure_pursuit_info.path[17] = p17;
    r2n_chassis.pure_pursuit_info.path[18] = p18;
    r2n_chassis.pure_pursuit_info.path[19] = p19;
    r2n_chassis.pure_pursuit_info.path[20] = p20;
    r2n_chassis.pure_pursuit_info.path[21] = p21;
    r2n_chassis.pure_pursuit_info.path[22] = p22;
    r2n_chassis.pure_pursuit_info.path[23] = p23;
    r2n_chassis.pure_pursuit_info.path[24] = p24;

    r2n_chassis.pure_pursuit_info.path[25] = p25;
    r2n_chassis.pure_pursuit_info.path[26] = p26;
    r2n_chassis.pure_pursuit_info.path[27] = p27;
    r2n_chassis.pure_pursuit_info.path[28] = p28;
    r2n_chassis.pure_pursuit_info.path[29] = p29;
    r2n_chassis.pure_pursuit_info.path[30] = p30;
    r2n_chassis.pure_pursuit_info.path[31] = p31;
    r2n_chassis.pure_pursuit_info.path[32] = p32;
    r2n_chassis.pure_pursuit_info.path[33] = p33;
    r2n_chassis.pure_pursuit_info.path[34] = p34;
    r2n_chassis.pure_pursuit_info.path[35] = p35;
    r2n_chassis.pure_pursuit_info.path[36] = p36;
    r2n_chassis.pure_pursuit_info.path[37] = p37;
    r2n_chassis.pure_pursuit_info.path[38] = p38;
    r2n_chassis.pure_pursuit_info.path[39] = p39;
    r2n_chassis.pure_pursuit_info.path[40] = p40;
    r2n_chassis.pure_pursuit_info.path[41] = p41;
    r2n_chassis.pure_pursuit_info.path[42] = p42;
    r2n_chassis.pure_pursuit_info.path[43] = p43;
    r2n_chassis.pure_pursuit_info.path[44] = p44;
    r2n_chassis.pure_pursuit_info.path[45] = p45;
    r2n_chassis.pure_pursuit_info.path[46] = p46;
    r2n_chassis.pure_pursuit_info.path[47] = p47;
    r2n_chassis.pure_pursuit_info.path[48] = p48;

    r2n_chassis.pure_pursuit_info.point_sum = 49;
    r2n_chassis.pure_pursuit_info.if_loop = true;

    /*
        can_core.init();
        Action.startUartReceiveIT();
        esp32_serial.startUartReceiveIT();
        ros_serial.startUartReceiveIT();
        esp32_serial.addsubscriber(&r1_remote);

        task_core.registerTask(0, &can_core);
        task_core.registerTask(3, &r1e_chassis);
        task_core.registerTask(2, &r1_remote);
        task_core.registerTask(4, &ros_serial);
        //task_core.registerTask(3, &test1);
        ros_serial.tx_frame_mat.data_length = 8;
        ros_serial.tx_frame_mat.frame_id = 0x01;
        */

    /*八期r2
     can_core.init();
     ros_serial.startUartReceiveIT();
     task_core.registerTask(0, &can_core);
     task_core.registerTask(4, &ros_serial);
     task_core.registerTask(3, &test1);
     ros_serial.tx_frame_mat.data_length = 8;
     ros_serial.tx_frame_mat.frame_id = 0x01;
     */

    osKernelStart();
}

void demo::process_data()
{
    // ros_serial.tx_frame_mat.data.msg_get[0] = m6020_shoot.real_angle;
    // m6020_shoot.target_angle = 144.0f;
    // data_chain.tx_frame_mat.data.msg_get[0] = Action.pose_data.world_pos_x;
    // data_chain.tx_frame_mat.data.msg_get[1] = Action.pose_data.world_pos_y;
    // m6020_shoot.rpm_pid.PID_SetParameters(ros_serial.rx_frame_mat.data.msg_get[1], ros_serial.rx_frame_mat.data.msg_get[2], ros_serial.rx_frame_mat.data.msg_get[3]);

    /*
        r2n_chassis.point_track_info.target_distan = 50.0f;
        if (data_chain.rx_frame_mat.frame_id == 1 && data_chain.rx_frame_mat.data_length == 8)
        {
            r2n_chassis.point_track_info.target_x = data_chain.rx_frame_mat.data.msg_get[0];
            r2n_chassis.point_track_info.target_y = data_chain.rx_frame_mat.data.msg_get[1] - 2000.0f;
        }
        */

    /*
        if (change_flag)
        {
            r2n_chassis.line_track_info.target_line_initpoint = p1;
            r2n_chassis.line_track_info.target_line = v1;
            if (r2n_chassis.line_track_info.tangent_dis < 36.0f && r2n_chassis.line_track_info.tangent_dis != 0.0f)
            {
                change_flag = false;
            }
        }
        else
        {
            r2n_chassis.line_track_info.target_line_initpoint = p2;
            r2n_chassis.line_track_info.target_line = v2;
        }
        */
    // r2n_chassis.distan_pid.PID_SetParameters(ros_serial.rx_frame_mat.data.msg_get[1], ros_serial.rx_frame_mat.data.msg_get[2], ros_serial.rx_frame_mat.data.msg_get[3]);
}
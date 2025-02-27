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

m3508p m3508_shooter(1, &hcan1), m3508_pitch(2, &hcan1);
// m6020s m6020_test(4, &hcan2);
// vesc vesc_test(1, &hcan1);
TaskManager task_core;
CanManager can_core;
//  shoot_xbox shoot_control(&m3508_shooter, &m3508_pitch);
RC9Protocol debug(cdc), esp32(uart, &huart3), position_port(uart, &huart4);

position position_test;

RoboChassis m4(mecanum_chassis);
servo mg996_left(&htim9, TIM_CHANNEL_1), mg996_right(&htim9, TIM_CHANNEL_2);

// swerve4 swerve_test(&vesc_test, &m6020_test);
tb6612 right_back(&htim2, TIM_CHANNEL_1, &htim8, GPIOC, GPIO_PIN_1, GPIOC, GPIO_PIN_0), right_front(&htim2, TIM_CHANNEL_2, &htim4, GPIOC, GPIO_PIN_3, GPIOC, GPIO_PIN_2), left_back(&htim2, TIM_CHANNEL_3, &htim3, GPIOF, GPIO_PIN_2, GPIOF, GPIO_PIN_1), left_front(&htim2, TIM_CHANNEL_4, &htim1, GPIOF, GPIO_PIN_4, GPIOF, GPIO_PIN_3);

debug_xbox xbox_test;
chassis_debug_xbox xbox_ctrl(1.6f, 1.2f);

demo test2;

chassis_info m4_chassis_info = {0.03f, 0.17f, 0.3f, 0.0f};

catcher_fsm claw_test;

extern "C" void
pshoot_setup(void)
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
    mg996_right.init();
    // test2.add_chassis(&m4);
    m4.config(m4_chassis_info);
    m4.pointtrack_config(1.96f, 0.0f, 0.86f, 0.0f, 3.0f, 0.005f, 0.0f);
    m4.yawadjuster_config(0.032f, 0.0f, 0.04f, 0.0f, 2.0f, 0.5f, 0.0f);

    debug.initQueue();

    esp32.startUartReceiveIT();
    position_port.startUartReceiveIT();

    position_test.addport(&position_port);

    //task_core.registerTask(4, &right_front);
    task_core.registerTask(4, &right_back);
    task_core.registerTask(5, &left_back);
    //task_core.registerTask(5, &left_front);
    task_core.registerTask(5, &claw_test);

    task_core.registerTask(8, &debug);
    task_core.registerTask(1, &can_core);
    //task_core.registerTask(3, &m4);
    task_core.registerTask(3, &xbox_ctrl);
    //m4.add4_motors(&left_front, &right_front, &right_back, &left_back);
    m4.add_imu(&position_test);
    // xbox_test.addport(&esp32);
    xbox_ctrl.addport(&esp32);
    // mg996_left.set_ccr(left_up); // 146 最低位,80最高位
    // mg996_right.set_ccr(right_up);

    claw_test.add_servo(&mg996_left, &mg996_right);

    // test2.add_IO(&xbox_test, &debug);
    xbox_ctrl.add_chassis(&m4);
    test2.addport(&debug);
    xbox_ctrl.claw = &claw_test;
    // debug.rcninit(3);
    //  mg996_left.set_ccr(150);

    // mknum_test.heading_pid.error > -0.01f;
    // mknum_test.heading_pid.error < 0.01f;
    osKernelStart();
}

void demo::process_data()
{
    currentTick = HAL_GetTick(); // ???? tick ?

    // ??????????????
    elapsedTime = (float)(currentTick - previousTick);

    if (elapsedTime >= 1) // ??????? 1 ??
    {
        previousTick = currentTick; // ????? tick ?
                                    //  ????????????
                                    //  ??,LED???
    }

    Vector2D vel(0.0f, 0.0f);
    // set_RobotVel(vel, 3);
    //  set_RobotW(0.0f, 3);
    // yaw_TurnTo(-get_input_mapvalue() * full_angle, 3);

    // pid_send_debuginfo(get_input_mapvalue() * full_angle, get_yaw());
    //  CCR = (uint32_t)test_ccr;
    //  CCR2 = (uint32_t)test2_ccr;
    //  mg996_right.set_ccr(CCR);
    //  mg996_left.set_ccr(CCR2);

    float test_datas[3] = {0.1f, 0.2f, 0.3f};
    sendFloatData(1, test_datas, 3);
    // claw_test.throw_ball();
}

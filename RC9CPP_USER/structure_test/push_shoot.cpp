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

// m3508p m3508_shooter(1, &hcan1), m3508_pitch(2, &hcan1);
// m6020s m6020_test(4, &hcan2);
// vesc vesc_test(1, &hcan1);
TaskManager task_core;
// CanManager can_core;
//  shoot_xbox shoot_control(&m3508_shooter, &m3508_pitch);
RC9Protocol debug(cdc), esp32(uart, &huart3);

HWT101CT yaw_sensor(&huart2);
RoboChassis m4(mecanum_chassis);
servo mg996_left(&htim9, TIM_CHANNEL_1), mg996_right(&htim9, TIM_CHANNEL_2);

// swerve4 swerve_test(&vesc_test, &m6020_test);
tb6612 right_back(&htim2, TIM_CHANNEL_1, &htim8, GPIOC, GPIO_PIN_1, GPIOC, GPIO_PIN_0), left_front(&htim2, TIM_CHANNEL_2, &htim4, GPIOC, GPIO_PIN_3, GPIOC, GPIO_PIN_2), right_front(&htim2, TIM_CHANNEL_3, &htim3, GPIOF, GPIO_PIN_2, GPIOF, GPIO_PIN_1), left_back(&htim2, TIM_CHANNEL_4, &htim1, GPIOF, GPIO_PIN_4, GPIOF, GPIO_PIN_3);

debug_xbox xbox_test;

demo test2;

extern "C" void pshoot_setup(void)
{
    // box_test.rcninit(2);
    // esp32_serial.rcninit(1);

    // esp32_serial.msgbuff_pub.init("xboxbuff", SYN, &esp32_serial);
    // box_test.buff_sub.init("xboxbuff", SYN, &box_test);
    // can_core.init();
    right_front.init();
    right_back.init();
    left_front.init();
    left_back.init();
    mg996_left.init();
    mg996_right.init();
    test2.add_chassis(&m4);
    yaw_sensor.startUartReceiveIT();
    debug.initQueue();
    esp32.startUartReceiveIT();
    //  debug.startUartReceiveIT();
    //  debug.addsubscriber(&test2);
    //  test2.addsubscriber(&debug);
    // esp32_serial.startUartReceiveIT();
    //  wwwwfdi_test.startUartReceiveIT();
    //  esp32_serial.addsubscriber(&resxbox);
    //  task_core.registerTask(0, &can_core);
    //  task_core.registerTask(1, &vesc_test);
    //  task_core.registerTask(1, &box_test);
    //  task_core.registerTask(2, &mknum_test);
    //  task_core.registerTask(4, &right_front);
    //  task_core.registerTask(4, &right_back);
    //  task_core.registerTask(5, &left_front);
    //  task_core.registerTask(5, &left_back);
    //  task_core.registerTask(6, &odom_test);
    //  task_core.registerTask(6, &resxbox);
    task_core.registerTask(8, &debug);
    task_core.registerTask(7, &test2);
    task_core.registerTask(3, &m4);
    m4.add4_motors(&right_front, &right_back, &left_back, &left_front);
    xbox_test.addport(&esp32);
    mg996_left.set_ccr(left_up); // 146 最低位,80最高位
    mg996_right.set_ccr(right_up);

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

    // sendFloatData(1, &test_data, 1);
    Vector2D vel(6.0f, 7.0f);
    set_RobotVel(vel, 3);
    set_RobotW(2.0f, 3);
}

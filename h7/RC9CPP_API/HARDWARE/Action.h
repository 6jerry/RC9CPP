#ifndef ACTION_H
#define ACTION_H

#ifdef __cplusplus
extern "C"
{
#endif
#include "Serial_device.h"
#include "math.h"
#include "imu.h"
#ifdef __cplusplus
}
#endif
#ifdef __cplusplus
#define FRAME_HEAD_ACTION_0 0x0D
#define FRAME_HEAD_ACTION_1 0x0A
#define FRAME_TAIL_ACTION_0 0x0A
#define FRAME_TAIL_ACTION_1 0x0D

#define DATA_LENGTH_ACTION 24  // 数据段长度（24字节）
#define FRAME_LENGTH_ACTION 28 // 整个包的长度（头+数据+尾）

/*机器人坐标和全局坐标定义:机器人前方为y轴正方向，机器人左侧为x轴正方向，角速度顺时针为正，角度范围为顺时针0到180，逆时针0到-180*/

typedef struct action_frame
{
    uint8_t temp_buff[DATA_LENGTH_ACTION] = {0};

    uint8_t frame_head[2] = {0};
    union data
    {
        float msg_get[DATA_LENGTH_ACTION / 4] = {0.0f}; // 用于浮点数的接收
        uint8_t buff_msg[DATA_LENGTH_ACTION];           // 用于字节流的接收
    } data;
    uint8_t frame_end[2] = {0};
} action_frame;
typedef struct action_info_
{
    float last_pos_x = 0.0f;
    float last_pos_y = 0.0f;
    float last_pos_z = 0.0f;

    float lastspeed_x = 0.0f;
    float lastspeed_y = 0.0f;

    float now_pos_x = 0.0f;
    float now_pos_y = 0.0f;
    float now_pos_z = 0.0f;

    float delta_pos_x = 0.0f;
    float delta_pos_y = 0.0f;
    float delta_pos_z = 0.0f;

    float pos_z_sum = 0.0f;
    float pos_x_sum = 0.0f;
    float pos_y_sum = 0.0f; // mm

    float Dx = 0.0f;
    float Dy = 0.0f;

} action_info_;
typedef struct action_install_pos_
{
    float delta_x = 0.0f;
    float delta_y = 0.0f;

} action_install_pos_;

typedef struct speed_vector_
{
    float magnitude = 0.0f; // 大小和方向
    float direction = 0.0f;

} speed_vector_;

typedef struct pose_data_
{
    float robot_speed_x = 0.0f; // m/s
    float robot_speed_y = 0.0f;

    float yaw_rad = 0.0f;
    float yaw_angle = 0.0f;

    float world_pos_x = 0.0f; // m
    float world_pos_y = 0.0f;
    float world_speed_x = 0.0f;
    float world_speed_y = 0.0f;

    float acc_x = 0.0f;
    float acc_y = 0.0f;

    speed_vector_ speed_vector; // 机器人速度矢量

} pose_data_;

class action : public SerialDevice, public imu
{

public:                              // 面向用户的友好接口函数
    void restart();                  // 重定位，以当前位置为坐标原点
    void relocate(float x, float y); // 重定位，以传入的坐标为坐标原点

    // 获取一些常用信息
    float get_pose_x();
    float get_pose_y();
    float get_heading() override;

    float get_yaw_rad() override;
    float get_speedx();
    float get_speedy();

    Vector2D get_world_pos() override;

    void imu_rst() override;
    void imu_relocate(float x, float y, float angle) override;
    void Update_ACTION(void);
    void relocateAll(float angle, float pos_x, float pos_y);

public:
    // 内部状态机的状态
    enum RxState
    {
        WAITING_FOR_HEADER_0,
        WAITING_FOR_HEADER_1,
        RECEIVING_DATA,
        WAITING_FOR_TAIL_0,
        WAITING_FOR_TAIL_1
    } state_;
    uint8_t rxIndex_; // 当前数据字节索引
    action_info_ action_info;
    action_install_pos_ action_install_pos;
    bool if_inverse_install = false; // action是不是反着装的，专门给九期r2打的补丁，服了
    bool if_init = true;
    uint8_t init_count = 0;
    uint8_t byte_get = 0;
    float previous_world_pos_x = 0.0f;
    float previous_world_pos_y = 0.0f;
    float delta_time = 0.0f;
    uint32_t previous_time = 0; // 上一次的时间戳，单位为ms
    void calculateWorldSpeed();

public:
    void Update_Action_gl_position(float value[6]);
    void handleReceiveData(uint8_t byte);
    action(UART_HandleTypeDef *huart, float install_delta_x, float install_delta_y, bool if_inverse_install_);
    action_frame rx_action_frame;
    pose_data_ pose_data;
};

#endif

#endif
#include "Action.h"

action::action(UART_HandleTypeDef *huart, float install_delta_x, float install_delta_y, bool if_inverse_install_) : SerialDevice(huart), state_(WAITING_FOR_HEADER_0), rxIndex_(0), if_inverse_install(if_inverse_install_)
{
    action_install_pos.delta_x = install_delta_x;
    action_install_pos.delta_y = install_delta_y;
}

void action::handleReceiveData(uint8_t byte)
{
    byte_get = byte;
    switch (state_)
    {
    case WAITING_FOR_HEADER_0:
        if (byte == FRAME_HEAD_ACTION_0) // 检查包头的第一个字节
        {
            state_ = WAITING_FOR_HEADER_1;
        }
        break;

    case WAITING_FOR_HEADER_1:
        if (byte == FRAME_HEAD_ACTION_1) // 检查包头的第二个字节
        {
            state_ = RECEIVING_DATA;
            rxIndex_ = 0; // 重置接收索引
        }
        else if (byte == FRAME_HEAD_ACTION_0)
        {
            /* code */
        }
        else
        {
            state_ = WAITING_FOR_HEADER_0; // 不是正确的包头，重新开始
        }
        break;

    case RECEIVING_DATA:
        rx_action_frame.temp_buff[rxIndex_++] = byte; // 保存接收到的字节
        if (rxIndex_ >= DATA_LENGTH_ACTION)           // 接收到24字节数据
        {
            state_ = WAITING_FOR_TAIL_0;
        }
        break;

    case WAITING_FOR_TAIL_0:
        if (byte == FRAME_TAIL_ACTION_0) // 检查包尾的第一个字节
        {
            state_ = WAITING_FOR_TAIL_1;
        }
        else
        {
            state_ = WAITING_FOR_HEADER_0; // 包尾不匹配，重新开始
        }
        break;

    case WAITING_FOR_TAIL_1:
        if (byte == FRAME_TAIL_ACTION_1) // 检查包尾的第二个字节
        {
            // 包接收完毕，解析数据
            for (uint8_t i = 0; i < DATA_LENGTH_ACTION; i++)
            {
                rx_action_frame.data.buff_msg[i] = rx_action_frame.temp_buff[i];
            }
            Update_Action_gl_position(rx_action_frame.data.msg_get);
        }
        state_ = WAITING_FOR_HEADER_0; // 处理完毕后重新等待新包
        break;

    default:
        state_ = WAITING_FOR_HEADER_0;
        break;
    }
}

void action::Update_Action_gl_position(float value[6])
{
    if (if_init)
    {
        init_count++;
        if (init_count > 6)
        {
            restart();
            if_init = false;
        }
    }
    action_info.last_pos_x = action_info.now_pos_x;
    action_info.last_pos_y = action_info.now_pos_y;
    action_info.last_pos_z = action_info.now_pos_z;

    if (if_inverse_install)
    {
        action_info.now_pos_z = value[0];
        action_info.now_pos_x = value[3];
        action_info.now_pos_y = value[4];
    }
    else
    {
        action_info.pos_z_sum = value[0];
        action_info.pos_x_sum = value[3];
        action_info.pos_y_sum = -value[4];
    }

    // action_info.delta_pos_x = action_info.now_pos_x - action_info.last_pos_x;
    // action_info.delta_pos_y = action_info.now_pos_y - action_info.last_pos_y;
    // action_info.delta_pos_z = action_info.now_pos_z - action_info.last_pos_z;

    // action_info.pos_z_sum += action_info.delta_pos_z;

    pose_data.yaw_angle = -action_info.pos_z_sum;

    pose_data.yaw_rad = pose_data.yaw_angle * 0.01745f;
    // action_info.pos_x_sum += action_info.delta_pos_x;
    // action_info.pos_y_sum += action_info.delta_pos_y;

    // 机器人坐标系下的静态安装偏移量转换到世界坐标
    action_info.Dx = cos(pose_data.yaw_rad) * action_install_pos.delta_x - sin(pose_data.yaw_rad) * action_install_pos.delta_y;
    action_info.Dy = sin(pose_data.yaw_rad) * action_install_pos.delta_x + cos(pose_data.yaw_rad) * action_install_pos.delta_y;
    // 减去安装偏移量
    pose_data.world_pos_x = action_info.pos_x_sum;
    pose_data.world_pos_y = action_info.pos_y_sum;
    // calculateWorldSpeed();
}

void action::restart()
{

    // action_info.pos_z_sum = 0.0f;
    // action_info.pos_x_sum = action_install_pos.delta_x;
    // action_info.pos_y_sum = action_install_pos.delta_y;
    Update_ACTION();
}

void action::imu_rst()
{
    restart();
}

void action::imu_relocate(float x, float y, float angle)
{
    relocateAll(angle, x * 1000.0f, -y * 1000.0f);
}

void action::Update_ACTION(void)
{
    uint8_t data[8] = "ACT0";

    HAL_UART_Transmit(huart_, data, 4, 1);
}

void action::relocateAll(float angle, float pos_x, float pos_y)
{
    // 构造命令："ACTA" + angle + pos_x + pos_y（float, 小端序发送）
    uint8_t buf[4 + 3 * sizeof(float)];
    buf[0] = 'A';
    buf[1] = 'C';
    buf[2] = 'T';
    buf[3] = 'A';
    // 使用union确保按IEEE754小端格式打包
    union
    {
        float f;
        uint8_t b[4];
    } u;

    u.f = angle;
    memcpy(&buf[4], u.b, 4);
    u.f = pos_x;
    memcpy(&buf[8], u.b, 4);
    u.f = pos_y;
    memcpy(&buf[12], u.b, 4);

    // 发送校准命令
    HAL_UART_Transmit(huart_, buf, sizeof(buf), 1);
}

void action::relocate(float x, float y)
{
    action_info.pos_x_sum = x + action_info.Dx;
    action_info.pos_y_sum = y + action_info.Dy;
}
// 计算世界坐标系下的速度
void action::calculateWorldSpeed()
{
    // 获取当前时间戳
    uint32_t current_time = HAL_GetTick(); // 获取当前时间，单位ms

    if (previous_time != 0)
    { // 确保上一次时间不为0
      // 计算时间差，转换为秒
        delta_time = float(current_time - previous_time) / 1000.0f;

        if (delta_time > 0.0f)
        { // 避免除零
            // 计算位置增量
            float delta_x = (pose_data.world_pos_x - previous_world_pos_x) / 1000.0f;
            float delta_y = (pose_data.world_pos_y - previous_world_pos_y) / 1000.0f;

            // 计算速度
            pose_data.world_speed_x = delta_x / delta_time;
            pose_data.world_speed_y = delta_y / delta_time;

            // pose_data.acc_x = (pose_data.world_speed_x - action_info.lastspeed_x) / delta_time;

            // pose_data.acc_y = (pose_data.world_speed_y - action_info.lastspeed_y) / delta_time;
        }
    }

    // 更新上一次的位置和时间
    previous_world_pos_x = pose_data.world_pos_x;
    previous_world_pos_y = pose_data.world_pos_y;
    previous_time = current_time;

    // action_info.lastspeed_x = pose_data.world_speed_x;
    // action_info.lastspeed_y = pose_data.world_speed_y;
}

float action::get_pose_x()
{
    return pose_data.world_pos_x;
}

float action::get_pose_y()
{
    return pose_data.world_pos_y;
}

float action::get_heading()
{
    return pose_data.yaw_angle;
}

float action::get_yaw_rad()
{
    return pose_data.yaw_rad;
}

float action::get_speedx()
{
    return pose_data.world_speed_x;
}
float action::get_speedy()
{
    return pose_data.world_speed_y;
}

Vector2D action::get_world_pos()
{
    Vector2D pos;
    pos.x = pose_data.world_pos_x / 1000.0f;
    pos.y = pose_data.world_pos_y / 1000.0f;

    return pos;
}
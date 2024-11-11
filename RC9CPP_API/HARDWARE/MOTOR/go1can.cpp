#include "go1can.h"

uint32_t go1can::generateCanExtId(
    uint8_t module_id,       // 模块ID (2位)，取值范围 0-3
    bool is_send,            // 发送/接收指示位，发送为0，接收为1
    uint8_t data_mode,       // 数据内容指示位 (2位)，取值范围 0-3
    uint8_t ctrl_mode,       // 控制模式 (8位)，根据手册定义取值范围 0-255
    uint8_t motor_id,        // 目标电机ID (4位)，取值范围 0-15
    uint8_t motor_ctrl_mode, // 电机控制模式 (3位)，取值范围 0-7
    uint8_t reserved_bits    // 预留位 (12位)，通常设为0
)
{

    uint32_t ext_id = 0;
    ext_id |= (module_id & 0x3) << 27;       // 模块ID (2位)
    ext_id |= (is_send ? 0 : 1) << 26;       // 发送/接收指示位 (1位)
    ext_id |= (data_mode & 0x3) << 24;       // 数据内容指示位 (2位)
    ext_id |= (ctrl_mode & 0xFF) << 16;      // 控制模式 (8位)
    ext_id |= (motor_id & 0xF) << 8;         // 目标电机ID (4位)
    ext_id |= (motor_ctrl_mode & 0x7) << 12; // 电机控制模式 (3位)
    ext_id |= reserved_bits & 0x1FF;         // 预留位 (9位)

    return ext_id;
}
go1can::go1can(uint8_t can_id, CAN_HandleTypeDef *hcan_) : CanDevice(GO1, hcan_, can_id)
{
}

uint32_t go1can::getExtid_loadData(uint8_t *data)
{
    if (mode_change_flag)
    {
        extid = generateCanExtId(
            3,    // 模块ID
            true, // 是否发送
            0,    // 数据模式
            11,   // 控制模式
            0,    // 目标电机ID
            1     // 电机控制模式
        );
        // 设置参数

        K_pos_int = 0;
        K_spd_int = 256;

        data[3] = (K_pos_int >> 8) & 0xFF; // 刚度系数高字节
        data[2] = K_pos_int & 0xFF;        // 刚度系数低字节
        data[1] = (K_spd_int >> 8) & 0xFF; // 阻尼系数高字节
        data[0] = K_spd_int & 0xFF;        // 阻尼系数低字节
    }
    else
    {
        extid = generateCanExtId(
            3,    // 模块ID
            true, // 是否发送
            0,    // 数据模式
            13,   // 控制模式
            0,    // 目标电机ID
            1     // 电机控制模式
        );
        // 设置参数

        data[5] = (wset >> 8) & 0xFF;
        data[4] = wset & 0xFF;
    }
    mode_change_flag = (mode_change_flag + 1) % 2;

    return extid;
}
void go1can::can_update(uint8_t can_RxData[8])
{
}
void go1can::EXT_ID_update(uint32_t ext_id)
{

}

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
go1can::go1can(uint8_t can_id, CAN_HandleTypeDef *hcan_, int16_t K_pos_int_, int16_t K_spd_int_) : CanDevice(GO1, hcan_, can_id), K_pos_int(K_pos_int_), K_spd_int(K_spd_int_)
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
void go1can::EXT_update(uint32_t ext_id, uint8_t can_RxData[8])
{
    rec_go1_id = parseCanExtId(extid);
    if (rec_go1_id.data_mode == 2) // 返回那两个参数
    {
        K_pos_rec = combine_bytes(can_RxData[3], can_RxData[2]);
        K_sqd_rec = combine_bytes(can_RxData[1], can_RxData[0]);
    }
    else if (rec_go1_id.low_byte_1 == -128 && rec_go1_id.data_mode == 1) // 该帧是错误帧
    {
        switch (rec_go1_id.ctrl_mode)
        {
        case 0:
            if_error_flag = false;
            error = none;
            break;
        case 1:
            if_error_flag = true;
            error = overheat;
            break;
        case 2:
            if_error_flag = true;
            error = overcurrent;
            break;
        case 3:
            if_error_flag = true;
            error = overvoltage;
            break;
        case 4:
            if_error_flag = true;
            error = encoder_error;
            break;
        default:
            break;
        }
    }
    else // 常规模式
    {
        if_error_flag = false;
        temp = rec_go1_id.low_byte_1;
        prec = combine_four_bytes(can_RxData[3], can_RxData[2], can_RxData[1], can_RxData[0]);
        wrec = combine_bytes(can_RxData[5], can_RxData[4]);
        trec = combine_bytes(can_RxData[7], can_RxData[6]);
    }
}
int16_t go1can::combine_bytes(uint8_t high_byte, uint8_t low_byte)
{
    return (int16_t)((high_byte << 8) | low_byte);
}
int32_t go1can::combine_four_bytes(uint8_t byte3, uint8_t byte2, uint8_t byte1, uint8_t byte0)
{
    return (int32_t)((byte3 << 24) | (byte2 << 16) | (byte1 << 8) | byte0);
}
ParsedExtId go1can::parseCanExtId(uint32_t ext_id)
{
    ParsedExtId parsed;

    // 解析各个字段
    parsed.module_id = (ext_id >> 27) & 0x3;       // 模块ID (2位)
    parsed.is_send = !((ext_id >> 26) & 0x1);      // 发送/接收指示位 (1位)，发送为0，接收为1
    parsed.data_mode = (ext_id >> 24) & 0x3;       // 数据内容指示位 (2位)
    parsed.ctrl_mode = (ext_id >> 16) & 0xFF;      // 控制模式 (8位)
    parsed.motor_id = (ext_id >> 8) & 0xF;         // 目标电机ID (4位)
    parsed.motor_ctrl_mode = (ext_id >> 12) & 0x7; // 电机控制模式 (3位)
    // parsed.reserved_bit = (ext_id >> 8) & 0x1;     // 预留位 (1位)
    parsed.low_byte_1 = ext_id & 0xFF; // 低位1 (8位)

    return parsed;
}

void go1can::process_data()
{
    uint8_t data[8] = {0};
    switch (mode)
    {
    case go1_setting:
        if (!if_error_flag)
        {
            extid = generateCanExtId(
                3,    // 模块ID
                true, // 是否发送
                0,    // 数据模式
                11,   // 控制模式
                0,    // 目标电机ID
                1     // 电机控制模式
            );
            data[3] = (K_pos_int >> 8) & 0xFF; // 刚度系数高字节
            data[2] = K_pos_int & 0xFF;        // 刚度系数低字节
            data[1] = (K_spd_int >> 8) & 0xFF; // 阻尼系数高字节
            data[0] = K_spd_int & 0xFF;        // 阻尼系数低字节

            // CAN_Send(extid, true, data);

            extid = generateCanExtId(
                3,    // 模块ID
                true, // 是否发送
                0,    // 数据模式
                12,   // 控制模式
                0,    // 目标电机ID
                1     // 电机控制模式
            );
            CAN_Send(extid, true, data);

            if (K_pos_int == K_pos_rec && K_spd_int == K_sqd_rec)
            {
                // 已经成功设置
                mode = go1_standby;
            }
            mode = speed;
        }
        break;

    case go1_standby:
        if (!if_error_flag)
        {
            extid = generateCanExtId(
                3,    // 模块ID
                true, // 是否发送
                0,    // 数据模式
                10,   // 控制模式
                0,    // 目标电机ID
                1     // 电机控制模式
            );
            /*
            data[5] = (wset >> 8) & 0xFF;
            data[4] = wset & 0xFF;
            data[7] = (tset >> 8) & 0xFF;
            data[6] = tset & 0xFF;
            data[0] = (uint8_t)(pset & 0xFF);
            data[1] = (uint8_t)((pset >> 8) & 0xFF);
            data[2] = (uint8_t)((pset >> 16) & 0xFF);
            data[3] = (uint8_t)((pset >> 24) & 0xFF);
            */
            CAN_Send(extid, true, data);
        }
        break;

    case speed:
        extid = generateCanExtId(
            3,    // 模块ID
            true, // 是否发送
            0,    // 数据模式
            11,   // 控制模式
            0,    // 目标电机ID
            1     // 电机控制模式
        );
        data[3] = (K_pos_int >> 8) & 0xFF; // 刚度系数高字节
        data[2] = K_pos_int & 0xFF;        // 刚度系数低字节
        data[1] = (K_spd_int >> 8) & 0xFF; // 阻尼系数高字节
        data[0] = K_spd_int & 0xFF;        // 阻尼系数低字节

        CAN_Send(extid, true, data);
        extid = generateCanExtId(
            3,    // 模块ID
            true, // 是否发送
            0,    // 数据模式
            10,   // 控制模式
            0,    // 目标电机ID
            1     // 电机控制模式
        );
        data[5] = (wset >> 8) & 0xFF;
        data[4] = wset & 0xFF;
        CAN_Send(extid, true, data);
        break;

    case pos:
        extid = generateCanExtId(
            3,    // 模块ID
            true, // 是否发送
            0,    // 数据模式
            10,   // 控制模式
            0,    // 目标电机ID
            1     // 电机控制模式
        );
        data[0] = (uint8_t)(pset & 0xFF);
        data[1] = (uint8_t)((pset >> 8) & 0xFF);
        data[2] = (uint8_t)((pset >> 16) & 0xFF);
        data[3] = (uint8_t)((pset >> 24) & 0xFF);
        CAN_Send(extid, true, data);
        break;

    default:
        break;
    }
}
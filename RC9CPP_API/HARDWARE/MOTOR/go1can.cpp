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
go1can::go1can(uint8_t can_id, CAN_HandleTypeDef *hcan_, float r_, float w0_, float kp_, float kd_, float b0_) : CanDevice(GO1, hcan_, can_id), speed_ladrc(r_, w0_, b0_, kp_, kd_, 8.0f)
{
    extid = generateCanExtId(
        3,    // 模块ID
        true, // 是否发送
        0,    // 数据模式
        10,   // 控制模式
        0,    // 目标电机ID
        1     // 电机控制模式
    );
    // speed_plan.speed_pulse_plan_start(6.28f, 0.8f, 11.0f, -20.0f, -1.04645f, 0.0f, 0.0f);
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

        real_speed = ((float)wrec / 256.0f) * 9.4787f;
        real_pos = ((float)prec / 32768.0f) * 0.9926f;
        real_t = ((float)trec / 256.0f);
        real_speed = -((real_speed / 60.0f) * 6.28f);
        if (real_pos >= 0.0f)
        {
            relative_angle = -(0.4415f + real_pos);
        }
        else
        {
            relative_angle = -0.4415f - real_pos;
        }
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

    case standby:
        if (!if_error_flag)
        {
            CAN_Send(extid, true, data); // 力矩为0
        }
        break;

    case speed:

        target_speed = speed_plan.speed_pulse_plan_setpos(relative_angle);
        if (speed_plan.now_state == uniform_acc)
        {
            acc_t = -(((0.167767f + icc) * speed_plan.plan_info.acc) / 6.33f);
        }
        else if (speed_plan.now_state == uniform)
        {
            acc_t = -0.31f;
        }
        else if (speed_plan.now_state == uniform_dec)
        {

            acc_t = -(((0.220767f) * speed_plan.plan_info.dec) / 6.33f);
            if (real_speed < 0.0f)
            {
                acc_t = 0.0f;
            }
        }

        // rpm_pid.setpoint = target_rpm; //-0.727046 go1零点
        // speed_pid.setpoint = target_speed;
        // target_t = -0.79f * cos(relative_angle) + rpm_pid.PID_Compute(real_speed);
        target_t = -0.79f * cos(relative_angle) - ff_feed * (speed_ladrc.ladrc_Compute(exp, real_speed) / 6.33f);
        tset = (int16_t)(target_t * 256.0f); // 转矩要加符号
        data[7] = (tset >> 8) & 0xFF;        // 补偿系数0.785
        data[6] = tset & 0xFF;
        CAN_Send(extid, true, data); //-0.66075
        // 转动惯量0.17476666666
        break;

    case pos_many:

        break;

    default:
        break;
    }
}

float go1can::get_rpm()
{
    return real_speed;
}
void go1can::set_rpm(float power_motor_rpm)
{
    target_rpm = power_motor_rpm;
}

void go1can::set_rpm_ff(float power_motor_rpm, float ff)
{
    target_rpm = power_motor_rpm;
    target_ff = ff;
}


#ifndef GO1CAN_H
#define GO1CAN_H

#ifdef __cplusplus
extern "C"
{
#endif

#include <math.h>
#include "can_device.h"
#include "TaskManager.h"
#include "motor.h"
#include "pid.h"

#ifdef __cplusplus
}
#endif

#ifdef __cplusplus
enum go1_mode
{
    go1_setting,
    go1_standby, // 速度为0，待命中
    speed,       // 自带的速度环
    pos,         // 自带的位置环
    pid_speed,   // 自己搓的速度pid
    pid_pos,     // 自己搓的位置pid
    error,
    go1_relax // 零力矩
};
enum go1_error
{
    overheat,
    overcurrent,
    overvoltage,
    overpressure,
    encoder_error,
    none
};

struct ParsedExtId
{
    uint8_t module_id = 0;
    bool is_send = false;
    uint8_t data_mode = 0;
    uint8_t ctrl_mode = 0;
    uint8_t motor_id = 0;
    uint8_t motor_ctrl_mode = 0;
    // uint8_t reserved_bit; // 预留位 (1位)
    int8_t low_byte_1; // 低位1 (8位)
};
class go1can : public CanDevice, public ITaskProcessor, public power_motor
{
private:
    uint32_t generateCanExtId(
        uint8_t module_id,        // 模块ID (2位)，取值范围 0-3
        bool is_send,             // 发送/接收指示位，发送为0，接收为1
        uint8_t data_mode,        // 数据内容指示位 (2位)，取值范围 0-3
        uint8_t ctrl_mode,        // 控制模式 (8位)，根据手册定义取值范围 0-255
        uint8_t motor_id,         // 目标电机ID (4位)，取值范围 0-15
        uint8_t motor_ctrl_mode,  // 电机控制模式 (3位)，取值范围 0-7
        uint8_t reserved_bits = 0 // 预留位
    );

    uint8_t mode_change_flag = 0; // 发一次系数设置帧，发一次速度设置帧
    uint32_t extid = 0;
    int16_t K_pos_int = 0, K_spd_int = 0, K_pos_rec = -1, K_sqd_rec;

public:
    uint32_t getExtid_loadData(uint8_t *data);
    int32_t pset = 0, prec = 0;
    int16_t wset = 0, tset = 0, wrec = 0, trec = 0; // 发送的虚拟角速度值
    int8_t temp = 0;                                // 电机温度，摄氏度
    void can_update(uint8_t can_RxData[8]);
    go1can(uint8_t can_id, CAN_HandleTypeDef *hcan_, float kp_, float ki_, float kd_);
    void EXT_update(uint32_t ext_id, uint8_t can_RxData[8]) override;
    void process_data();
    int16_t combine_bytes(uint8_t high_byte, uint8_t low_byte);
    int32_t combine_four_bytes(uint8_t byte3, uint8_t byte2, uint8_t byte1, uint8_t byte0);
    ParsedExtId parseCanExtId(uint32_t ext_id);
    ParsedExtId rec_go1_id;
    go1_mode mode = go1_standby;
    go1_error error = none;
    bool if_error_flag = false;

    float real_speed = 0.0f, real_pos = 0.0f, real_t = 0.0f, target_rpm = 0.0f, target_t = 0.0f;

    float get_rpm();
    void set_rpm(float power_motor_rpm);

    bool switch_go1_mode(go1_mode target_mode);

    pid rpm_pid;
};

#endif

#endif



#ifndef GO1CAN_H
#define GO1CAN_H

#ifdef __cplusplus
extern "C"
{
#endif

#include <math.h>
#include "can_device.h"
#include "TaskManager.h"

#ifdef __cplusplus
}
#endif

#ifdef __cplusplus

class go1can : public CanDevice, public ITaskProcessor
{
private:
    uint32_t generateCanExtId(
        uint8_t module_id,        // 模块ID (2位)，取值范围 0-3
        bool is_send,             // 发送/接收指示位，发送为0，接收为1
        uint8_t data_mode,        // 数据内容指示位 (2位)，取值范围 0-3
        uint8_t ctrl_mode,        // 控制模式 (8位)，根据手册定义取值范围 0-255
        uint8_t motor_id,         // 目标电机ID (4位)，取值范围 0-15
        uint8_t motor_ctrl_mode,  // 电机控制模式 (3位)，取值范围 0-7
        uint8_t reserved_bits = 0 // 预留位 (12位)，通常设为0
    );

    uint8_t mode_change_flag = 0; // 发一次系数设置帧，发一次速度设置帧
    uint32_t extid = 0;
    int16_t K_pos_int = 0, K_spd_int = 0;
    int16_t tset = 0, pset = 0;

public:
    uint32_t getExtid_loadData(uint8_t *data);
    int16_t wset = 0;
    void can_update(uint8_t can_RxData[8]);
    go1can(uint8_t can_id, CAN_HandleTypeDef *hcan_);
    void EXT_ID_update(uint32_t ext_id) override;
    void process_data();
};

#endif

#endif

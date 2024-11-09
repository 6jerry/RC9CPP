#ifndef CAN_DEVICE_H
#define CAN_DEVICE_H

#ifdef __cplusplus
extern "C"
{
#endif

    // #include <stm32f4xx_hal_can.h>
    // #include <stm32f407xx.h>

#include <stdbool.h>
#include "can.h"
#include "TaskManager.h"
#include "go1can.h"
#ifdef __cplusplus
}
#endif
#ifdef __cplusplus
#define MAX_INSTANCES 4 // 一条can上最多挂四个3508
enum CanDeviceType
{
    M3508,
    M2006,
    DM43,
    VESC,
    M6020,
    GO1
};

enum can_id
{
    m3508_id_1 = 0x201,
    m3508_id_2 = 0x202,
    m3508_id_3 = 0x203,
    m3508_id_4 = 0x204,

    gm6020_id_1 = 0x205,
    gm6020_id_2 = 0x206,
    gm6020_id_3 = 0x207,
    gm6020_id_4 = 0x208

};

class CanDevice
{
public:
    CAN_HandleTypeDef *hcan_;  // CAN 句柄
    CanDeviceType deviceType_; // 设备类型
    uint8_t can_id = 0;
    virtual int16_t motor_process(); // 给大疆用的接口，其他电机不要管

   

    virtual void can_update(uint8_t can_RxData[8]) = 0;

    static CanDevice *m3508_instances_can1[MAX_INSTANCES]; // 保存所有实例,供can管理者使用
    static CanDevice *m6020_instances_can1[MAX_INSTANCES];
    static int instanceCount_m3508_can1;

    static CanDevice *m3508_instances_can2[MAX_INSTANCES];
    static CanDevice *m6020_instances_can2[MAX_INSTANCES];
    static int instanceCount_m3508_can2;
    static int instanceCount_m6020_can1;
    static int instanceCount_m6020_can2;

    CanDevice(CanDeviceType deviceType_, CAN_HandleTypeDef *hcan_, uint8_t can_id);
};

class CanManager : public ITaskProcessor
{
private:
    void CAN1_Filter_Init(void);
    void CAN2_Filter_Init(void);
    CAN_TxHeaderTypeDef tx_message_1, tx_message_2;
    uint8_t send_buf1[8] = {0};
    uint8_t send_buf2[8] = {0};
    uint32_t msg_box1 = 0;
    uint32_t msg_box2 = 0;
    int8_t error_flag = 0;
    uint32_t generateCanExtId(
        uint8_t module_id,        // 模块ID (2位)，取值范围 0-3
        bool is_send,             // 发送/接收指示位，发送为0，接收为1
        uint8_t data_mode,        // 数据内容指示位 (2位)，取值范围 0-3
        uint8_t ctrl_mode,        // 控制模式 (8位)，根据手册定义取值范围 0-255
        uint8_t motor_id,         // 目标电机ID (4位)，取值范围 0-15
        uint8_t motor_ctrl_mode,  // 电机控制模式 (3位)，取值范围 0-7
        uint8_t reserved_bits = 0 // 预留位 (12位)，通常设为0
    );

public:
    void process_data();
    CanManager();
    void init();
    go1can* Go1;
    static uint8_t RxData1[8];
    static uint8_t RxData2[8];
};

#endif
#endif
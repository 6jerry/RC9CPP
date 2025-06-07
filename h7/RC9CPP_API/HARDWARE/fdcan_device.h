#ifndef FDCAN_DEVICE_H
#define FDCAN_DEVICE_H

#ifdef __cplusplus
extern "C"
{
#endif
#include "fdcan.h"
#include "sort_search.h"
#include "gpio.h"
#ifdef __cplusplus
}
#endif

#ifdef __cplusplus
#define MAX_CAN_DEVICES 32
#define BUS_COUNT 3 // 总线数量

enum CanFrameType
{
    CAN_FRAME_STD = 0,
    CAN_FRAME_EXT = 1
};

class CanDevice
{
public:
    FDCAN_HandleTypeDef *hcan_; // CAN 句柄
    uint32_t can_id_;           // 设备使用的 CAN ID
    CanFrameType frameType_;
    virtual void can_update(uint8_t rx_data[8]) = 0;

    static HAL_StatusTypeDef CAN_Send(uint32_t can_id, uint8_t is_extended, uint8_t data[8], FDCAN_HandleTypeDef *hcansend);
    // 中断统一分发入口，在 HAL_FDCAN_RxFifo0Callback 里调用
    static void dispatch(FDCAN_HandleTypeDef *hcan, FDCAN_RxHeaderTypeDef *rxh, uint8_t data[8]);

    CanDevice(FDCAN_HandleTypeDef *hcan, CanFrameType type, uint32_t can_id);
    static HAL_StatusTypeDef InitAllFiltersNoMask();

    static HAL_StatusTypeDef FDCAN_ConfigAllStdAndExt(FDCAN_HandleTypeDef *hfdcan);

private:
    // 两张表：标准帧／拓展帧
    // 为每路总线、每种帧类型各维护一个表
    static CanDevice *std_devs_[BUS_COUNT][MAX_CAN_DEVICES];
    static uint32_t std_ids_[BUS_COUNT][MAX_CAN_DEVICES];
    static uint8_t std_cnt_[BUS_COUNT];

    static CanDevice *ext_devs_[BUS_COUNT][MAX_CAN_DEVICES];
    static uint32_t ext_ids_[BUS_COUNT][MAX_CAN_DEVICES];
    static uint8_t ext_cnt_[BUS_COUNT];

    void registerSelf();
    // helper: 根据 hcan_ 返回 0..BUS_COUNT-1
    static int getBusIdx(FDCAN_HandleTypeDef *hcan);
};

#endif
#endif
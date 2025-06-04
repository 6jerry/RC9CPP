#include "fdcan_device.h"

// 各路 CAN、各帧类型的静态成员定义
CanDevice *CanDevice::std_devs_[BUS_COUNT][MAX_CAN_DEVICES] = {{0}};
uint32_t CanDevice::std_ids_[BUS_COUNT][MAX_CAN_DEVICES] = {{0}};
uint8_t CanDevice::std_cnt_[BUS_COUNT] = {0};

CanDevice *CanDevice::ext_devs_[BUS_COUNT][MAX_CAN_DEVICES] = {{0}};
uint32_t CanDevice::ext_ids_[BUS_COUNT][MAX_CAN_DEVICES] = {{0}};
uint8_t CanDevice::ext_cnt_[BUS_COUNT] = {0};

HAL_StatusTypeDef CanDevice::CAN_Send(uint32_t can_id, uint8_t is_extended, uint8_t data[8], FDCAN_HandleTypeDef *hcansend)
{
    FDCAN_TxHeaderTypeDef txHeader; // 定义一个 CAN 发送头结构体变量
                                    // 定义一个发送邮箱变量

    // 设置 CAN ID 和帧类型
    if (is_extended)
    {
        txHeader.IdType = FDCAN_EXTENDED_ID; // 设置为扩展帧
        txHeader.Identifier = can_id;        // 设置扩展 ID
    }
    else
    {
        txHeader.IdType = FDCAN_STANDARD_ID; // 设置为标准帧
        txHeader.Identifier = can_id;        // 设置标准 ID
    }

    txHeader.TxFrameType = FDCAN_DATA_FRAME; // 设置为数据帧
    txHeader.DataLength = FDCAN_DLC_BYTES_8; // 数据长度为 8 字节
    txHeader.FDFormat = FDCAN_CLASSIC_CAN;   // 不使用全局时间戳
    txHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    txHeader.MessageMarker = 0;
    txHeader.BitRateSwitch = FDCAN_BRS_OFF;
    txHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;

    // 调用 HAL_CAN_AddTxMessage 发送数据
    return HAL_FDCAN_AddMessageToTxFifoQ(hcansend, &txHeader, data);
}
void CanDevice::dispatch(FDCAN_HandleTypeDef *hcan, FDCAN_RxHeaderTypeDef *rxh, uint8_t data[8])
{
    int bus = getBusIdx(hcan);
    uint32_t id = rxh->Identifier;

    if (rxh->IdType == FDCAN_STANDARD_ID)
    {
        int idx = binarySearch(std_ids_[bus], std_cnt_[bus], id);
        if (idx >= 0)
        {
            std_devs_[bus][idx]->can_update(data);
        }
    }
    else
    {
        int idx = binarySearch(ext_ids_[bus], ext_cnt_[bus], id);
        if (idx >= 0)
        {
            ext_devs_[bus][idx]->can_update(data);
        }
    }
}

void CanDevice::registerSelf()
{
    int bus = getBusIdx(hcan_);
    if (frameType_ == CAN_FRAME_STD)
    {
        if (std_cnt_[bus] >= MAX_CAN_DEVICES)
            return;
        // 插入排序保持 std_ids_[bus] 有序
        int pos = std_cnt_[bus]++;
        while (pos > 0 && std_ids_[bus][pos - 1] > can_id_)
        {
            std_ids_[bus][pos] = std_ids_[bus][pos - 1];
            std_devs_[bus][pos] = std_devs_[bus][pos - 1];
            --pos;
        }
        std_ids_[bus][pos] = can_id_;
        std_devs_[bus][pos] = this;
    }
    else
    {
        if (ext_cnt_[bus] >= MAX_CAN_DEVICES)
            return;
        int pos = ext_cnt_[bus]++;
        while (pos > 0 && ext_ids_[bus][pos - 1] > can_id_)
        {
            ext_ids_[bus][pos] = ext_ids_[bus][pos - 1];
            ext_devs_[bus][pos] = ext_devs_[bus][pos - 1];
            --pos;
        }
        ext_ids_[bus][pos] = can_id_;
        ext_devs_[bus][pos] = this;
    }
}

CanDevice::CanDevice(FDCAN_HandleTypeDef *hcan, CanFrameType type, uint32_t can_id) : hcan_(hcan), frameType_(type), can_id_(can_id)
{
    registerSelf();
}
int CanDevice::getBusIdx(FDCAN_HandleTypeDef *hcan)
{
    if (hcan == &hfdcan1)
        return 0;
    if (hcan == &hfdcan2)
        return 1;
    // 默认第三路
    return 2;
}

HAL_StatusTypeDef CanDevice::InitAllFiltersNoMask()
{

    if (FDCAN_ConfigAllStdAndExt(&hfdcan2) != HAL_OK)
        return HAL_ERROR;
    if (FDCAN_ConfigAllStdAndExt(&hfdcan1) != HAL_OK)
        return HAL_ERROR;
    if (FDCAN_ConfigAllStdAndExt(&hfdcan3) != HAL_OK)
        return HAL_ERROR;
   
    HAL_FDCAN_Start(&hfdcan1); // 开启FDCAN
    HAL_FDCAN_Start(&hfdcan2);
    HAL_FDCAN_Start(&hfdcan3);
   
    HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
    HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
    HAL_FDCAN_ActivateNotification(&hfdcan3, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);

    

    return HAL_OK;
}

HAL_StatusTypeDef CanDevice::FDCAN_ConfigAllStdAndExt(FDCAN_HandleTypeDef *hfdcan)
{
    // 公共配置
    FDCAN_FilterTypeDef cfg = {0};
    cfg.FilterType = FDCAN_FILTER_MASK;         // Mask 模式
    cfg.FilterConfig = FDCAN_FILTER_TO_RXFIFO0; // 放入 RX FIFO0

    // 全接收：设置 ID1=0, ID2=0 (Mask=0 ⇒ 接收所有)
    cfg.FilterID1 = 0;
    cfg.FilterID2 = 0;

    // 标准帧
    cfg.IdType = FDCAN_STANDARD_ID;
    cfg.FilterIndex = 0;
    HAL_StatusTypeDef s0 = HAL_FDCAN_ConfigFilter(hfdcan, &cfg);

    // 扩展帧
    cfg.IdType = FDCAN_EXTENDED_ID;
    cfg.FilterIndex = 1;
    HAL_StatusTypeDef s1 = HAL_FDCAN_ConfigFilter(hfdcan, &cfg);

    return (s0 == HAL_OK && s1 == HAL_OK) ? HAL_OK : HAL_ERROR;
}

extern "C" void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    FDCAN_RxHeaderTypeDef rxh;
    uint8_t data[8];
    if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &rxh, data) != HAL_OK)
    {
        // 读取失败，跳过
        return;
    }

    // 交给 CanDevice 全局分发函数，自动选帧型、选总线、二分查找并调用 can_update
    CanDevice::dispatch(hfdcan, &rxh, data);
}
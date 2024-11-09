#include "can_device.h"
// 静态变量定义
CanDevice *CanDevice::m3508_instances_can1[MAX_INSTANCES] = {nullptr};
CanDevice *CanDevice::m3508_instances_can2[MAX_INSTANCES] = {nullptr};
int CanDevice::instanceCount_m3508_can1 = 0;
int CanDevice::instanceCount_m3508_can2 = 0;

CanDevice *CanDevice::m6020_instances_can1[MAX_INSTANCES] = {nullptr}; // M6020 实例
CanDevice *CanDevice::m6020_instances_can2[MAX_INSTANCES] = {nullptr};
int CanDevice::instanceCount_m6020_can1 = 0;
int CanDevice::instanceCount_m6020_can2 = 0;

uint8_t CanManager::RxData1[8] = {0};
uint8_t CanManager::RxData2[8] = {0};

int16_t
CanDevice::motor_process()
{
    return 0;
}

// 以上都是虚函数，不用在基类写具体的东西

CanDevice::CanDevice(CanDeviceType deviceType_, CAN_HandleTypeDef *hcan_, uint8_t can_id) : deviceType_(deviceType_), hcan_(hcan_), can_id(can_id)
{
    // 注册该can设备
    if (hcan_ == &hcan1)
    {
        switch (deviceType_)
        {
        case CanDeviceType::M3508:
            if (instanceCount_m3508_can1 < MAX_INSTANCES)
            {
                // 使用插入排序方法，根据 can_id 插入到合适位置
                int insertPos = instanceCount_m3508_can1;
                for (int i = 0; i < instanceCount_m3508_can1; ++i)
                {
                    if (m3508_instances_can1[i]->can_id > can_id)
                    {
                        insertPos = i;
                        break;
                    }
                }

                for (int i = instanceCount_m3508_can1; i > insertPos; --i)
                {
                    m3508_instances_can1[i] = m3508_instances_can1[i - 1];
                }

                m3508_instances_can1[insertPos] = this;
                instanceCount_m3508_can1++;
            }
            break;

        case CanDeviceType::M6020:
            if (instanceCount_m6020_can1 < MAX_INSTANCES)
            {
                int insertPos = instanceCount_m6020_can1;
                for (int i = 0; i < instanceCount_m6020_can1; ++i)
                {
                    if (m6020_instances_can1[i]->can_id > can_id)
                    {
                        insertPos = i;
                        break;
                    }
                }

                for (int i = instanceCount_m6020_can1; i > insertPos; --i)
                {
                    m6020_instances_can1[i] = m6020_instances_can1[i - 1];
                }

                m6020_instances_can1[insertPos] = this;
                instanceCount_m6020_can1++;
            }
            break;

        default:
            break;
        }
    }
    if (hcan_ == &hcan2)
    {
        switch (deviceType_)
        {
        case CanDeviceType::M3508:
            if (instanceCount_m3508_can2 < MAX_INSTANCES)
            {
                int insertPos = instanceCount_m3508_can2;
                for (int i = 0; i < instanceCount_m3508_can2; ++i)
                {
                    if (m3508_instances_can2[i]->can_id > can_id)
                    {
                        insertPos = i;
                        break;
                    }
                }

                for (int i = instanceCount_m3508_can2; i > insertPos; --i)
                {
                    m3508_instances_can2[i] = m3508_instances_can2[i - 1];
                }

                m3508_instances_can2[insertPos] = this;
                instanceCount_m3508_can2++;
            }
            break;

        case CanDeviceType::M6020:
            if (instanceCount_m6020_can2 < MAX_INSTANCES)
            {
                int insertPos = instanceCount_m6020_can2;
                for (int i = 0; i < instanceCount_m6020_can2; ++i)
                {
                    if (m6020_instances_can2[i]->can_id > can_id)
                    {
                        insertPos = i;
                        break;
                    }
                }

                for (int i = instanceCount_m6020_can2; i > insertPos; --i)
                {
                    m6020_instances_can2[i] = m6020_instances_can2[i - 1];
                }

                m6020_instances_can2[insertPos] = this;
                instanceCount_m6020_can2++;
            }
            break;

        default:
            break;
        }
    }
}

void CanManager::CAN1_Filter_Init(void)
{
    CAN_FilterTypeDef sFilterConfig;

    sFilterConfig.FilterBank = 0;                      // 使用第一个滤波器银行
    sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;  // 使用掩码模式，不过滤特定 ID
    sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT; // 使用 32 位滤波器
    sFilterConfig.FilterIdHigh = 0x0000;               // ID 高位为 0
    sFilterConfig.FilterIdLow = 0x0000;                // ID 低位为 0
    sFilterConfig.FilterMaskIdHigh = 0x0000;           // 掩码高位为 0（不筛选 ID）
    sFilterConfig.FilterMaskIdLow = 0x0000;            // 掩码低位为 0（不筛选 ID）
    sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0; // 将接收的消息放入 FIFO0
    sFilterConfig.FilterActivation = ENABLE;           // 启用滤波器
    sFilterConfig.SlaveStartFilterBank = 14;           // 如果使用双 CAN，CAN2 从滤波器 14 开始

    if (HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig) != HAL_OK)
    {
        /* Filter configuration Error */
        Error_Handler();
    }

    if (HAL_CAN_Start(&hcan1) != HAL_OK)
    {
        /* Start Error */
        Error_Handler();
    }

    /*##-4- Activate CAN RX notification #######################################*/
    if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
    {
        /* Start Error */
        Error_Handler();
    }

    TxHeader.ExtId = CAN_TxExtId; // 使用扩展 ID
    TxHeader.IDE = CAN_ID_EXT;    // 扩展帧
    TxHeader.RTR = CAN_RTR_DATA;  // 数据帧
    TxHeader.DLC = 8;             // 数据长度
    TxHeader.TransmitGlobalTime = DISABLE;
}

void CanManager::CAN2_Filter_Init(void)
{
    CAN_FilterTypeDef sFilterConfig;

    sFilterConfig.FilterBank = 14;                     // 使用第十四个滤波器银行（CAN2 起始）
    sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;  // 使用掩码模式，不过滤特定 ID
    sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT; // 使用 32 位滤波器
    sFilterConfig.FilterIdHigh = 0x0000;               // ID 高位为 0
    sFilterConfig.FilterIdLow = 0x0000;                // ID 低位为 0
    sFilterConfig.FilterMaskIdHigh = 0x0000;           // 掩码高位为 0（不筛选 ID）
    sFilterConfig.FilterMaskIdLow = 0x0000;            // 掩码低位为 0（不筛选 ID）
    sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0; // 将接收的消息放入 FIFO0
    sFilterConfig.FilterActivation = ENABLE;           // 启用滤波器
    sFilterConfig.SlaveStartFilterBank = 14;           // 如果使用双 CAN，CAN2 从滤波器 14 开始

    if (HAL_CAN_ConfigFilter(&hcan2, &sFilterConfig) != HAL_OK)
    {
        /* Filter configuration Error */
        Error_Handler();
    }

    if (HAL_CAN_Start(&hcan2) != HAL_OK)
    {
        /* Start Error */
        Error_Handler();
    }

    /*##-4- Activate CAN RX notification #######################################*/
    if (HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
    {
        /* Start Error */
        Error_Handler();
    }

    TxHeader.ExtId = CAN_TxExtId; // 使用扩展 ID
    TxHeader.IDE = CAN_ID_EXT;    // 扩展帧
    TxHeader.RTR = CAN_RTR_DATA;  // 数据帧
    TxHeader.DLC = 8;             // 数据长度
    TxHeader.TransmitGlobalTime = DISABLE;
}

void CanManager::init()
{
    CAN1_Filter_Init();
    CAN2_Filter_Init();
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
    HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);
}

CanManager::CanManager()
{
    tx_message_1.IDE = CAN_ID_STD;   // 报文的11位标准标识符CAN_ID_STD表示本报文是标准帧
    tx_message_1.RTR = CAN_RTR_DATA; // 报文类型标志RTR位CAN_ID_STD表示本报文的数据帧
    tx_message_1.DLC = 0x08;         // 数据段长度
    tx_message_1.TransmitGlobalTime = DISABLE;
    // 配置仲裁段和数据段
    tx_message_1.StdId = 0x00;

    tx_message_2.IDE = CAN_ID_STD;   // 报文的11位标准标识符CAN_ID_STD表示本报文是标准帧
    tx_message_2.RTR = CAN_RTR_DATA; // 报文类型标志RTR位CAN_ID_STD表示本报文的数据帧
    tx_message_2.DLC = 0x08;         // 数据段长度
    tx_message_2.TransmitGlobalTime = DISABLE;

    // 配置仲裁段和数据段
    tx_message_2.StdId = 0x00;
}
uint32_t CanManager::generateCanExtId(
    uint8_t module_id,       // 模块ID (2位)，取值范围 0-3
    bool is_send,            // 发送/接收指示位，发送为0，接收为1
    uint8_t data_mode,       // 数据内容指示位 (2位)，取值范围 0-3
    uint8_t ctrl_mode,       // 控制模式 (8位)，根据手册定义取值范围 0-255
    uint8_t motor_id,        // 目标电机ID (4位)，取值范围 0-15
    uint8_t motor_ctrl_mode, // 电机控制模式 (3位)，取值范围 0-7
    uint8_t reserved_bits    // 预留位 (12位)，通常设为0
)
{
    // 检查输入值是否在合理范围内
    if (module_id > 3 || data_mode > 3 || motor_id > 15 || motor_ctrl_mode > 7 || ctrl_mode > 255)
    {
        // 可以在这里设置错误处理
        return 0xFFFFFFFF; // 返回一个无效ID用于表示错误
    }

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

void CanManager::process_data()
{
    // 如果 CAN1 上有 m3508 设备
    if (CanDevice::instanceCount_m3508_can1 > 0)
    {
        for (int i = 0; i < CanDevice::instanceCount_m3508_can1; ++i)
        {
            int16_t temp_vcurrent = CanDevice::m3508_instances_can1[i]->motor_process();
            send_buf1[2 * i] = (uint8_t)(temp_vcurrent >> 8);
            send_buf1[2 * i + 1] = (uint8_t)temp_vcurrent;
        }
        tx_message_1.StdId = 0x200;
        if (HAL_CAN_AddTxMessage(&hcan1, &tx_message_1, send_buf1, &msg_box1) == HAL_ERROR)
        {
            error_flag = 1;
            // Failed to add message to the transmit mailbox
        }
    }

    // 如果 CAN1 上有 m6020 设备
    if (CanDevice::instanceCount_m6020_can1 > 0)
    {
        for (int i = 0; i < CanDevice::instanceCount_m6020_can1; ++i)
        {
            int16_t temp_vcurrent = CanDevice::m6020_instances_can1[i]->motor_process();
            send_buf1[2 * i] = (uint8_t)(temp_vcurrent >> 8);
            send_buf1[2 * i + 1] = (uint8_t)temp_vcurrent;
        }
        tx_message_1.StdId = 0x1FF;
        if (HAL_CAN_AddTxMessage(&hcan1, &tx_message_1, send_buf1, &msg_box1) == HAL_ERROR)
        {
            error_flag = 1;
        }
    }

    // 如果 CAN2 上有 m3508 设备
    if (CanDevice::instanceCount_m3508_can2 > 0)
    {
        for (int i = 0; i < CanDevice::instanceCount_m3508_can2; ++i)
        {
            int16_t temp_vcurrent2 = CanDevice::m3508_instances_can2[i]->motor_process();
            send_buf2[2 * i] = (uint8_t)(temp_vcurrent2 >> 8);
            send_buf2[2 * i + 1] = (uint8_t)temp_vcurrent2;
        }
        tx_message_2.StdId = 0x200;
        if (HAL_CAN_AddTxMessage(&hcan2, &tx_message_2, send_buf2, &msg_box2) != HAL_OK)
        {
            error_flag = 1;
        }
    }

    // 如果 CAN2 上有 m6020 设备
    if (CanDevice::instanceCount_m6020_can2 > 0)
    {
        for (int i = 0; i < CanDevice::instanceCount_m6020_can2; ++i)
        {
            int16_t temp_vcurrent2 = CanDevice::m6020_instances_can2[i]->motor_process();
            send_buf2[2 * i] = (uint8_t)(temp_vcurrent2 >> 8);
            send_buf2[2 * i + 1] = (uint8_t)temp_vcurrent2;
        }
        tx_message_2.StdId = 0x1FF;
        if (HAL_CAN_AddTxMessage(&hcan2, &tx_message_2, send_buf2, &msg_box2) != HAL_OK)
        {
            error_flag = 1;
        }
    }

    // 清空缓冲区
    for (int i = 0; i <= 7; i++)
    {
        send_buf2[i] = 0;
        send_buf1[i] = 0;
    }
    // 向 CAN1 发送一帧示例的控制帧，目标电机 ID 为 0，控制模式 1，设置期望速度
    CAN_TxHeaderTypeDef controlTxMessage;
    uint8_t controlData[8] = {0};
    uint32_t controlMailbox;

    uint32_t mod_id = 3;
    uint8_t ID = 0;
    int16_t send_speed = 180;

    // 构造扩展帧 ID (29 位)
    controlTxMessage.ExtId = Go1->getExtid_loadData(controlData);
    controlTxMessage.IDE = CAN_ID_EXT;   // 使用扩展帧
    controlTxMessage.RTR = CAN_RTR_DATA; // 数据帧
    controlTxMessage.DLC = 8;            //

    // 发送控制帧到 CAN1
    if (HAL_CAN_AddTxMessage(&hcan1, &controlTxMessage, controlData, &msg_box2) != HAL_OK)
    {
        error_flag = 1;
        // 发送失败处理
    }
}

extern "C" void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    if (hcan == &hcan1)
    {
        CAN_RxHeaderTypeDef RxHeader1;

        HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &RxHeader1, CanManager::RxData1);
        // 保持 M3508 的接收逻辑
        switch (RxHeader1.StdId)
        {
        case m3508_id_1:
            if (CanDevice::m3508_instances_can1[0] != nullptr)
            {
                CanDevice::m3508_instances_can1[0]->can_update(CanManager::RxData1);
            }
            break;
        case m3508_id_2:
            if (CanDevice::m3508_instances_can1[1] != nullptr)
            {
                CanDevice::m3508_instances_can1[1]->can_update(CanManager::RxData1);
            }
            break;
        case m3508_id_3:
            if (CanDevice::m3508_instances_can1[2] != nullptr)
            {
                CanDevice::m3508_instances_can1[2]->can_update(CanManager::RxData1);
            }
            break;
        case m3508_id_4:
            if (CanDevice::m3508_instances_can1[3] != nullptr)
            {
                CanDevice::m3508_instances_can1[3]->can_update(CanManager::RxData1);
            }
            break;

        // 增加 M6020 的接收逻辑
        case gm6020_id_1:
            if (CanDevice::m6020_instances_can1[0] != nullptr)
            {
                CanDevice::m6020_instances_can1[0]->can_update(CanManager::RxData1);
            }
            break;
        case gm6020_id_2:
            if (CanDevice::m6020_instances_can1[1] != nullptr)
            {
                CanDevice::m6020_instances_can1[1]->can_update(CanManager::RxData1);
            }
            break;
        case gm6020_id_3:
            if (CanDevice::m6020_instances_can1[2] != nullptr)
            {
                CanDevice::m6020_instances_can1[2]->can_update(CanManager::RxData1);
            }
            break;
        case gm6020_id_4:
            if (CanDevice::m6020_instances_can1[3] != nullptr)
            {
                CanDevice::m6020_instances_can1[3]->can_update(CanManager::RxData1);
            }
            break;

        default:
            break;
        }
    }
    else if (hcan == &hcan2)
    {
        CAN_RxHeaderTypeDef RxHeader2;
        HAL_CAN_GetRxMessage(&hcan2, CAN_RX_FIFO0, &RxHeader2, CanManager::RxData2);
        // 保持 M3508 的接收逻辑
        switch (RxHeader2.StdId)
        {
        case m3508_id_1:
            if (CanDevice::m3508_instances_can2[0] != nullptr)
            {
                CanDevice::m3508_instances_can2[0]->can_update(CanManager::RxData2);
            }
            break;
        case m3508_id_2:
            if (CanDevice::m3508_instances_can2[1] != nullptr)
            {
                CanDevice::m3508_instances_can2[1]->can_update(CanManager::RxData2);
            }
            break;
        case m3508_id_3:
            if (CanDevice::m3508_instances_can2[2] != nullptr)
            {
                CanDevice::m3508_instances_can2[2]->can_update(CanManager::RxData2);
            }
            break;
        case m3508_id_4:
            if (CanDevice::m3508_instances_can2[3] != nullptr)
            {
                CanDevice::m3508_instances_can2[3]->can_update(CanManager::RxData2);
            }
            break;

        // 增加 M6020 的接收逻辑
        case gm6020_id_1:
            if (CanDevice::m6020_instances_can2[0] != nullptr)
            {
                CanDevice::m6020_instances_can2[0]->can_update(CanManager::RxData2);
            }
            break;
        case gm6020_id_2:
            if (CanDevice::m6020_instances_can2[1] != nullptr)
            {
                CanDevice::m6020_instances_can2[1]->can_update(CanManager::RxData2);
            }
            break;
        case gm6020_id_3:
            if (CanDevice::m6020_instances_can2[2] != nullptr)
            {
                CanDevice::m6020_instances_can2[2]->can_update(CanManager::RxData2);
            }
            break;
        case gm6020_id_4:
            if (CanDevice::m6020_instances_can2[3] != nullptr)
            {
                CanDevice::m6020_instances_can2[3]->can_update(CanManager::RxData2);
            }
            break;

        default:
            break;
        }
    }
}

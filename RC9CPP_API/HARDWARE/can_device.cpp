#include "can_device.h"
// 静态变量定义
CanDevice *CanDevice::m3508_instances_can1[MAX_INSTANCES] = {nullptr};
CanDevice *CanDevice::m3508_instances_can2[MAX_INSTANCES] = {nullptr};
CanDevice *CanDevice::go1_instances_can1[MAX_INSTANCES] = {nullptr};
CanDevice *CanDevice::go1_instances_can2[MAX_INSTANCES] = {nullptr};
uint8_t CanDevice::instanceCount_m3508_can1 = 0;
uint8_t CanDevice::instanceCount_m3508_can2 = 0;

uint8_t CanDevice::instanceCount_go1_can1 = 0;
uint8_t CanDevice::instanceCount_go1_can2 = 0;

CanDevice *CanDevice::m6020_instances_can1[MAX_INSTANCES] = {nullptr}; // M6020 实例
CanDevice *CanDevice::m6020_instances_can2[MAX_INSTANCES] = {nullptr};
uint8_t CanDevice::instanceCount_m6020_can1 = 0;
uint8_t CanDevice::instanceCount_m6020_can2 = 0;
CanDevice *CanDevice::vesc_instances_can1[MAX_INSTANCES] = {nullptr};
CanDevice *CanDevice::vesc_instances_can2[MAX_INSTANCES] = {nullptr};
uint8_t CanDevice::instanceCount_vesc_can1 = 0; // 初始化为0
uint8_t CanDevice::instanceCount_vesc_can2 = 0; // 初始化为0

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

                m3508_instances_can1[can_id - 1] = this;
                instanceCount_m3508_can1++;
            }
            break;

        case CanDeviceType::M6020:
            if (instanceCount_m6020_can1 < MAX_INSTANCES)
            {

                m6020_instances_can1[can_id - 1] = this;
                instanceCount_m6020_can1++;
            }
            break;

        case CanDeviceType::GO1:
            if (instanceCount_go1_can1 < MAX_INSTANCES)
            {

                go1_instances_can1[can_id - 1] = this;
                instanceCount_go1_can1++;
            }
            break;

        case CanDeviceType::VESC:
            if (instanceCount_vesc_can1 < MAX_INSTANCES)
            {

                vesc_instances_can1[can_id - 1] = this;
                instanceCount_vesc_can1++;
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

                m3508_instances_can2[can_id - 1] = this;
                instanceCount_m3508_can2++;
            }
            break;

        case CanDeviceType::M6020:
            if (instanceCount_m6020_can2 < MAX_INSTANCES)
            {

                m6020_instances_can2[can_id - 1] = this;
                instanceCount_m6020_can2++;
            }
            break;

        case CanDeviceType::GO1:
            if (instanceCount_go1_can2 < MAX_INSTANCES)
            {

                go1_instances_can2[can_id - 1] = this;
                instanceCount_go1_can2++;
            }
            break;

        case CanDeviceType::VESC:
            if (instanceCount_vesc_can2 < MAX_INSTANCES)
            {

                vesc_instances_can2[can_id - 1] = this;
                instanceCount_vesc_can2++;
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

void CanManager::process_data()
{
    // 如果 CAN1 上有 m3508 设备
    if (CanDevice::instanceCount_m3508_can1 > 0)
    {
        for (int i = 0; i < MAX_INSTANCES; ++i)
        {
            if (CanDevice::m3508_instances_can1[i] != nullptr)
            {
                int16_t temp_vcurrent = CanDevice::m3508_instances_can1[i]->motor_process();
                send_buf1[2 * i] = (uint8_t)(temp_vcurrent >> 8);
                send_buf1[2 * i + 1] = (uint8_t)temp_vcurrent;
            }
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
        for (int i = 0; i < MAX_INSTANCES; ++i)
        {
            if (CanDevice::m6020_instances_can1[i] != nullptr)
            {
                int16_t temp_vcurrent = CanDevice::m6020_instances_can1[i]->motor_process();
                send_buf1[2 * i] = (uint8_t)(temp_vcurrent >> 8);
                send_buf1[2 * i + 1] = (uint8_t)temp_vcurrent;
            }
        }
        tx_message_1.StdId = 0x1FE;
        if (HAL_CAN_AddTxMessage(&hcan1, &tx_message_1, send_buf1, &msg_box1) == HAL_ERROR)
        {
            error_flag = 1;
        }
    }

    // 如果 CAN2 上有 m3508 设备
    if (CanDevice::instanceCount_m3508_can2 > 0)
    {
        for (int i = 0; i < MAX_INSTANCES; ++i)
        {
            if (CanDevice::m3508_instances_can2[i] == nullptr)
            {
                int16_t temp_vcurrent2 = CanDevice::m3508_instances_can2[i]->motor_process();
                send_buf2[2 * i] = (uint8_t)(temp_vcurrent2 >> 8);
                send_buf2[2 * i + 1] = (uint8_t)temp_vcurrent2;
            }
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
        for (int i = 0; i < MAX_INSTANCES; ++i)
        {
            if (CanDevice::m6020_instances_can2[i] == nullptr)
            {
                int16_t temp_vcurrent2 = CanDevice::m6020_instances_can2[i]->motor_process();
                send_buf2[2 * 3] = (uint8_t)(temp_vcurrent2 >> 8);
                send_buf2[2 * 3 + 1] = (uint8_t)temp_vcurrent2;
            }
        }
        tx_message_2.StdId = 0x1FE;
        if (HAL_CAN_AddTxMessage(&hcan2, &tx_message_2, send_buf2, &msg_box2) != HAL_OK)
        {
            error_flag = 1;
        }
    }
    /*uint8_t vesc_tx_buf[8] = {0}; // 数据缓冲区，8字节
    CAN_TxHeaderTypeDef vesc_tx_message;

    // 配置 CAN 帧 ID
    vesc_tx_message.ExtId = (CAN_CMD_SET_CURRENT << 8) | VESC_ID;
    vesc_tx_message.IDE = CAN_ID_EXT;   // 扩展帧
    vesc_tx_message.RTR = CAN_RTR_DATA; // 数据帧
    vesc_tx_message.DLC = 8;            // 数据帧长度为8字节

    // 目标电流（单位：安培），转换为 VESC 使用的毫安
    int32_t current_in_milliamp = (int32_t)(0);

    // 数据格式：
    // Byte 0: 命令类型 (CMD ID)
    // Byte 1~4: 电流值 (int32，单位：mA)
    vesc_tx_buf[0] = (current_in_milliamp >> 24) & 0xFF;
    vesc_tx_buf[1] = (current_in_milliamp >> 16) & 0xFF;
    vesc_tx_buf[2] = (current_in_milliamp >> 8) & 0xFF;
    vesc_tx_buf[3] = current_in_milliamp & 0xFF;

    if (HAL_CAN_AddTxMessage(&hcan1, &vesc_tx_message, vesc_tx_buf, &msg_box1) == HAL_ERROR)
    {
        error_flag = 1;
    }*/
    // 清空缓冲区
    for (int i = 0; i <= 7; i++)
    {
        send_buf2[i] = 0;
        send_buf1[i] = 0;
    }
}

extern "C" void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    if (hcan == &hcan1)
    {
        CAN_RxHeaderTypeDef RxHeader1;

        HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &RxHeader1, CanManager::RxData1);
        if (RxHeader1.IDE == CAN_ID_STD)
        {
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
        else
        {

            if (RxHeader1.ExtId >= 0x900 && RxHeader1.ExtId <= 0x908) // vesc电调的id范围
            {
                uint8_t vesc_id = RxHeader1.ExtId & 0xFF;
                // current = (int16_t)((CanManager ::RxData1[4] << 8) | CanManager ::RxData1[5]); // 电流，要乘个0.1
                //  erpm = (int32_t)((CanManager ::RxData1[0] << 24) | (CanManager ::RxData1[1] << 16) | (CanManager ::RxData1[2] << 8) | CanManager ::RxData1[3]); // 电器转速，记得除以电机的极对数

                // rrpm = (float)erpm / 7.0f;
                // rcurrent = (float)current * 0.1f;

                switch (vesc_id)
                {
                case 1:
                    if (CanDevice::vesc_instances_can1[0] != nullptr)
                    {
                        CanDevice::vesc_instances_can1[0]->can_update(CanManager::RxData1);
                    }
                    /* code */
                    break;

                case 2:
                    if (CanDevice::vesc_instances_can1[1] != nullptr)
                    {
                        CanDevice::vesc_instances_can1[1]->can_update(CanManager::RxData1);
                    }
                    break;

                case 3:
                    if (CanDevice::vesc_instances_can1[2] != nullptr)
                    {
                        CanDevice::vesc_instances_can1[2]->can_update(CanManager::RxData1);
                    }
                    break;

                case 4:
                    if (CanDevice::vesc_instances_can1[3] != nullptr)
                    {
                        CanDevice::vesc_instances_can1[3]->can_update(CanManager::RxData1);
                    }
                    break;

                default:
                    break;
                }
            }

            else // go1电机的id范围
            {
                uint8_t extgo1_motor_id = (RxHeader1.ExtId >> 8) & 0xF;
                switch (extgo1_motor_id)
                {
                case 0:
                    if (CanDevice::go1_instances_can1[0] != nullptr)
                    {
                        CanDevice::go1_instances_can1[0]->EXT_update(RxHeader1.ExtId, CanManager ::RxData1);
                    }
                    break;

                case 1:
                    if (CanDevice::go1_instances_can1[1] != nullptr)
                    {
                        CanDevice::go1_instances_can1[1]->EXT_update(RxHeader1.ExtId, CanManager ::RxData1);
                    }
                    break;

                case 2:
                    if (CanDevice::go1_instances_can1[2] != nullptr)
                    {
                        CanDevice::go1_instances_can1[2]->EXT_update(RxHeader1.ExtId, CanManager ::RxData1);
                    }
                    break;

                case 3:
                    if (CanDevice::go1_instances_can1[3] != nullptr)
                    {
                        CanDevice::go1_instances_can1[3]->EXT_update(RxHeader1.ExtId, CanManager ::RxData1);
                    }
                    break;

                default:
                    break;
                }
            }
        }
    }
    else if (hcan == &hcan2)
    {
        CAN_RxHeaderTypeDef RxHeader2;
        HAL_CAN_GetRxMessage(&hcan2, CAN_RX_FIFO0, &RxHeader2, CanManager::RxData2);
        if (RxHeader2.IDE == CAN_ID_STD)
        {
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
                if (CanDevice::m6020_instances_can2[0] != nullptr)
                {
                    CanDevice::m6020_instances_can2[0]->can_update(CanManager::RxData2);
                }
                break;

            default:
                break;
            }
        }
        else
        {
            if (RxHeader2.ExtId >= 0x900 && RxHeader2.ExtId <= 0x908) // vesc电调的id范围
            {
                uint8_t vesc_id = RxHeader2.ExtId & 0xFF;
                // current = (int16_t)((CanManager ::RxData1[4] << 8) | CanManager ::RxData1[5]); // 电流，要乘个0.1
                //  erpm = (int32_t)((CanManager ::RxData1[0] << 24) | (CanManager ::RxData1[1] << 16) | (CanManager ::RxData1[2] << 8) | CanManager ::RxData1[3]); // 电器转速，记得除以电机的极对数

                // rrpm = (float)erpm / 7.0f;
                // rcurrent = (float)current * 0.1f;

                switch (vesc_id)
                {
                case 1:
                    if (CanDevice::vesc_instances_can2[0] != nullptr)
                    {
                        CanDevice::vesc_instances_can2[0]->can_update(CanManager::RxData2);
                    }
                    /* code */
                    break;

                case 2:
                    if (CanDevice::vesc_instances_can2[1] != nullptr)
                    {
                        CanDevice::vesc_instances_can2[1]->can_update(CanManager::RxData2);
                    }
                    break;

                case 3:
                    if (CanDevice::vesc_instances_can2[2] != nullptr)
                    {
                        CanDevice::vesc_instances_can2[2]->can_update(CanManager::RxData2);
                    }
                    break;

                case 4:
                    if (CanDevice::vesc_instances_can2[3] != nullptr)
                    {
                        CanDevice::vesc_instances_can2[3]->can_update(CanManager::RxData2);
                    }
                    break;

                default:
                    break;
                }
            }
            else
            {
                uint8_t ext_motor_id = (RxHeader2.ExtId >> 8) & 0xF;

                switch (ext_motor_id)
                {
                case 0:
                    if (CanDevice::go1_instances_can2[0] != nullptr)
                    {
                        CanDevice::go1_instances_can2[0]->EXT_update(RxHeader2.ExtId, CanManager ::RxData2);
                    }
                    break;

                case 1:
                    if (CanDevice::go1_instances_can2[1] != nullptr)
                    {
                        CanDevice::go1_instances_can2[1]->EXT_update(RxHeader2.ExtId, CanManager ::RxData2);
                    }
                    break;

                case 2:
                    if (CanDevice::go1_instances_can2[2] != nullptr)
                    {
                        CanDevice::go1_instances_can2[2]->EXT_update(RxHeader2.ExtId, CanManager ::RxData2);
                    }
                    break;

                case 3:
                    if (CanDevice::go1_instances_can2[3] != nullptr)
                    {
                        CanDevice::go1_instances_can2[3]->EXT_update(RxHeader2.ExtId, CanManager ::RxData2);
                    }
                    break;

                default:
                    break;
                }
            }
        }
    }
}
uint8_t CanManager::canid_2_mac(CAN_HandleTypeDef *hcan)
{
    if (hcan == &hcan1)
    {
        CAN_RxHeaderTypeDef RxHeader1;

        HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &RxHeader1, CanManager::RxData1);
        if (RxHeader1.IDE == CAN_ID_STD)
        {
            return RxHeader1.StdId - 0x200;
        }
        else
        {
            if (RxHeader1.ExtId >= 0x900 && RxHeader1.ExtId <= 0x908)
            {
                // vesc电调
                return (RxHeader1.ExtId & 0xFF) + 16;
            }
        }
    }
    else
    {
        CAN_RxHeaderTypeDef RxHeader2;
        HAL_CAN_GetRxMessage(&hcan2, CAN_RX_FIFO0, &RxHeader2, CanManager::RxData2);
        if (RxHeader2.IDE == CAN_ID_STD)
        {
            return (RxHeader2.StdId - 0x200) + 8;
        }
        else
        {
            if (RxHeader2.ExtId >= 0x900 && RxHeader2.ExtId <= 0x908)
            {
                // vesc电调
                return (RxHeader2.ExtId & 0xFF) + 20;
            }
        }
    }
}
/**
 * @brief 发送 CAN 数据帧
 * @param can_id CAN ID
 * @param is_extended 是否为扩展帧
 * @param data 要发送的数据
 * @return HAL 状态
 */
HAL_StatusTypeDef CanDevice::CAN_Send(uint32_t can_id, uint8_t is_extended, uint8_t data[8])
{
    CAN_TxHeaderTypeDef txHeader; // 定义一个 CAN 发送头结构体变量
    uint32_t txMailbox;           // 定义一个发送邮箱变量

    // 设置 CAN ID 和帧类型
    if (is_extended)
    {
        txHeader.IDE = CAN_ID_EXT; // 设置为扩展帧
        txHeader.ExtId = can_id;   // 设置扩展 ID
    }
    else
    {
        txHeader.IDE = CAN_ID_STD; // 设置为标准帧
        txHeader.StdId = can_id;   // 设置标准 ID
    }

    txHeader.RTR = CAN_RTR_DATA;           // 设置为数据帧
    txHeader.DLC = 8;                      // 数据长度为 8 字节
    txHeader.TransmitGlobalTime = DISABLE; // 不使用全局时间戳

    // 调用 HAL_CAN_AddTxMessage 发送数据
    return HAL_CAN_AddTxMessage(hcan_, &txHeader, data, &txMailbox);
}

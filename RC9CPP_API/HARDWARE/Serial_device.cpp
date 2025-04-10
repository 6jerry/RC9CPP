#include "Serial_device.h"

// 静态变量初始化
SerialDevice *SerialDevice::instances_[MAX_INSTANCES] = {nullptr};
SerialDevice *SerialDevice::cdc_instance = nullptr;
int SerialDevice::instanceCount_ = 0;
// TaskHandle_t SerialDevice::sendTaskHandle_ = nullptr;
// bool SerialDevice::sendTaskCreated_ = false;

// 构造函数：传入 UART 句柄，自动启动接收中断
SerialDevice::SerialDevice(UART_HandleTypeDef *huart, uart_type type_)
    : huart_(huart), type(type_)
{
    if (type == uart)
    {
        // 注册当前实例到全局数组
        if (instanceCount_ < MAX_INSTANCES)
        {
            registerInstance(this);
        }
    }
    else if (type == cdc)
    {
        registerCDCInstance(this);
    }

    // 自动启动 UART 接收中断
    // startUartReceiveIT();
}

// 注册当前实例
void SerialDevice::registerInstance(SerialDevice *instance)
{
    instances_[instanceCount_++] = instance;
}

void SerialDevice::registerCDCInstance(SerialDevice *instance)
{
    cdc_instance = instance;
}

// 启用 UART 接收中断
void SerialDevice::startUartReceiveIT()
{
    if (huart_ != nullptr)
    {
        HAL_UART_Receive_IT(huart_, rxBuffer_, RX_BUFFER_SIZE);
    }
}
// 全局回调函数：HAL 库调用该函数
extern "C" void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    uint8_t rxByte;
    // 获取 UART 接收的字节
    for (int i = 0; i < SerialDevice::instanceCount_; i++)
    {
        if (SerialDevice::instances_[i]->huart_ == huart)
        {
            rxByte = SerialDevice::instances_[i]->rxBuffer_[0]; // 读取接收缓冲区中的字
            SerialDevice::instances_[i]->handleReceiveData(rxByte);

            HAL_UART_Receive_IT(huart, SerialDevice::instances_[i]->rxBuffer_, RX_BUFFER_SIZE);
            return;
        }
    }
    for (int i = 0; i < SerialDevice::instanceCount_; i++)
    {
        HAL_UART_Receive_IT(huart, SerialDevice::instances_[i]->rxBuffer_, RX_BUFFER_SIZE);
    }
}

extern "C"
{
    // 这是 C 接口，允许从 C 调用
    void ProcessReceivedData(uint8_t *Buf, uint32_t Len)
    {
        if (SerialDevice::cdc_instance != nullptr)
        {
            for (int i = 0; i < Len; i++)
            {
                SerialDevice::cdc_instance->handleReceiveData(Buf[i]);
            }
        }
    }
}
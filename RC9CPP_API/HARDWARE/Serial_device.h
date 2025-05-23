#ifndef SERIAL_DEVICE_H
#define SERIAL_DEVICE_H
#ifdef __cplusplus
extern "C"
{
#endif
#include "FreeRTOS.h"
#include "usart.h"
#include "task.h"
#include "queue.h"
#include <cmsis_os.h>
#include <stdbool.h>
#include "crc_util.h"
#include "usbd_cdc_if.h"
#ifdef __cplusplus
}
#endif
#ifdef __cplusplus
#define MAX_INSTANCES 10 // 最多支持 10 个串口实例

#define RX_BUFFER_SIZE 1 // 接收缓冲区大小

enum uart_type
{
    uart,
    cdc

};

class SerialDevice
{
public:
    // 构造函数：传入 UART 句柄，自动启动接收中断
    SerialDevice(UART_HandleTypeDef *huart, uart_type type_ = uart);

    // 注册当前实例到全局实例数组中
    static void registerInstance(SerialDevice *instance);
    static void registerCDCInstance(SerialDevice *instance);

    virtual void handleReceiveData(uint8_t byte) = 0;
    void startUartReceiveIT();
    static SerialDevice *instances_[MAX_INSTANCES]; // 保存所有实例
    static int instanceCount_;
    UART_HandleTypeDef *huart_; // 保存 UART 句柄
    uint8_t rxBuffer_[RX_BUFFER_SIZE];
    static SerialDevice *cdc_instance; // 作为虚拟串口的实例

    uart_type type = uart;

protected:
    // static TaskHandle_t sendTaskHandle_; // 静态变量：发送任务句柄
    // static bool sendTaskCreated_;        // 静态变量：是否已经创建了发送任务
};
#endif

#endif // SERIAL_DEVICE_H
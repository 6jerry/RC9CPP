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
    SerialDevice(UART_HandleTypeDef *huart, uart_type type_ = uart);

    static void registerInstance(SerialDevice *instance);
    static void registerCDCInstance(SerialDevice *instance);

    virtual void handleReceiveData(uint8_t byte) = 0;
    void startUartReceiveIT();
    static SerialDevice *instances_[MAX_INSTANCES]; // 保存所有普通串口实例
    static int instanceCount_;
    UART_HandleTypeDef *huart_; // 保存 UART 句柄
    uint8_t rxBuffer_[RX_BUFFER_SIZE];
    static SerialDevice *cdc_instance; // 虚拟串口实例

    uart_type type = uart;

protected:
};
#endif

#endif // SERIAL_DEVICE_H

#ifndef IO_INTERRUPT_H
#define IO_INTERRUPT_H

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
#define MAX_GPIO_INSTANCES 10 // 最大GPIO实例数

class GPIODevice
{
public:
    // 构造函数：传入GPIO端口、引脚和触发类型
    GPIODevice();

    // 注册实例到全局数组
    static void registerInstance(GPIODevice *instance);

    // 纯虚函数：子类必须实现的电平变化处理
    virtual void handleInterrupt() = 0;
    virtual void add_io_interrupt(GPIO_TypeDef *port_, uint16_t pin_) = 0;

    // 静态成员
    static GPIODevice *instances_[MAX_GPIO_INSTANCES]; // 实例指针数组
    static int instanceCount_;                         // 实例计数器

    // 硬件相关参数
    GPIO_TypeDef *port; // GPIO端口
    uint16_t pin;       // GPIO引脚

private:
    // 硬件中断初始化（可根据需要扩展）
    void initGPIOInterrupt();
};

// 中断回调函数（需在外部链接到HAL库回调）
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);
#endif

#endif // GPIO_DEVICE_H

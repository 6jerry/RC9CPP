#ifndef CHECK_DEVICE_H
#define CHECK_DEVICE_H

#ifdef __cplusplus
extern "C" {
#endif
#include "tim.h"
#include <stdbool.h>
#ifdef __cplusplus
}
#endif

#ifdef __cplusplus
#define MAX_CHILD_INSTANCES 32 // 最多支持32个子类实例

class CheckDevice
{
public:
    CheckDevice();
    CheckDevice(TIM_HandleTypeDef *htim);
    static void registerChild(CheckDevice *child);
    static void timing_processing(); // 统一定时处理接口
    static TIM_HandleTypeDef *htim_; // 基类管理的统一定时器
    bool isActiveFlag = false; // 子类继承的标志位
    static void initTimer(); // 初始化定时器
    uint32_t count_tick = 0; //子类继承的计数器

protected:
    virtual void onTimerEvent(){} // 子类实现的定时事件处理
    
private:
    static CheckDevice *children_[MAX_CHILD_INSTANCES];
    static uint8_t childCount_;
};

#endif // __cplusplus
#endif // CHECK_DEVICE_H
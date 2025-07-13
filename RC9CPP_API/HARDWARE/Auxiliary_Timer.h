#ifndef AUXILIARY_TIMER_H
#define AUXILIARY_TIMER_H

#ifdef __cplusplus
extern "C" {
#endif
#include "tim.h"
#include "stdint.h"
#ifdef __cplusplus
}
#endif

#ifdef __cplusplus
#define MAX_CHILD_INSTANCES 32 // 最多支持32个子类实例

class auxiliary_timer
{
public:
    auxiliary_timer(); // 子类构造函数
    auxiliary_timer(TIM_HandleTypeDef *htim); // 基类构造函数
    static void registerChild(auxiliary_timer *child);
    static void timing_processing(); // 统一定时处理接口
    static TIM_HandleTypeDef *htim_; // 基类管理的统一定时器
    bool isActiveFlag = false; // 子类继承的标志位
    static void initTimer(); // 初始化定时器
    uint64_t tick_us_overflow = 0; //子类继承的微秒计数值
    float count_tick_ms = 0; //子类继承的毫秒计数值(通过微秒计数器计算)
    uint64_t last_tick_us = 0;
    float last_tick_ms = 0;

    /************detla ms 配套使用*************/
    void set_delta_ms();  
    float get_delta_ms(); 
    /*****************************************/

    /************detla us 配套使用**************/
    void set_delta_us();
    uint64_t get_delta_us();
    /*****************************************/
    
    virtual void onTimerEvent(){} // 子类实现的定时事件处理
private:
    static auxiliary_timer *children_[MAX_CHILD_INSTANCES];
    static uint8_t childCount_;
};

#endif // __cplusplus
#endif // AUXILIARY_TIMER_H
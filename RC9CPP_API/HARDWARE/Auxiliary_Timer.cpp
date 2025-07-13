#include "Auxiliary_Timer.h"

// 静态变量初始化
auxiliary_timer *auxiliary_timer::children_[MAX_CHILD_INSTANCES] = {nullptr};
uint8_t auxiliary_timer::childCount_ = 0;
TIM_HandleTypeDef *auxiliary_timer::htim_ = nullptr;

// 构造函数(子类)
auxiliary_timer::auxiliary_timer()
{
    if (childCount_ < MAX_CHILD_INSTANCES)
    {
        registerChild(this);
    }
}

// 构造函数(基类)
auxiliary_timer::auxiliary_timer(TIM_HandleTypeDef *htim)
{
	htim_ = htim;
}

// 注册子类实例
void auxiliary_timer::registerChild(auxiliary_timer *child)
{
    children_[childCount_++] = child;
}

// 初始化定时器
void auxiliary_timer::initTimer()
{
    HAL_TIM_Base_Start_IT(htim_);
}

//ms開始計數區間
void auxiliary_timer::set_delta_ms()
{
    last_tick_ms = count_tick_ms;
}

float auxiliary_timer::get_delta_ms()
{
    return count_tick_ms - last_tick_ms;
}

//us開始計數區間
void auxiliary_timer::set_delta_us()
{
    last_tick_us = tick_us_overflow * 1000 + auxiliary_timer::htim_->Instance->CNT;
}

uint64_t auxiliary_timer::get_delta_us()
{
    return tick_us_overflow * 1000 + auxiliary_timer::htim_->Instance->CNT;
}

// 更新所有子类标志位
void auxiliary_timer::timing_processing()
{
    for (int i = 0; i < childCount_; i++)
    {	
		children_[i]->tick_us_overflow++;
        children_[i]->count_tick_ms++;
        children_[i]->onTimerEvent(); // 可选：直接调用处理函数
    }
}

//// 定时器中断回调
//extern "C" void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
//{

//  /* USER CODE BEGIN Callback 0 */
//    if (auxiliary_timer::htim_ == htim)
//    {
//        auxiliary_timer::timing_processing();
//    }
//  /* USER CODE END Callback 0 */
//  if (htim->Instance == TIM8) {
//		HAL_IncTick();.
//  /* USER CODE BEGIN Callback 1 */

//  /* USER CODE END Callback 1 */
//}
#include "Check_Device.h"

// 静态变量初始化
CheckDevice *CheckDevice::children_[MAX_CHILD_INSTANCES] = {nullptr};
uint8_t CheckDevice::childCount_ = 0;
TIM_HandleTypeDef *CheckDevice::htim_ = nullptr;

// 构造函数(子类)
CheckDevice::CheckDevice()
{
    if (childCount_ < MAX_CHILD_INSTANCES)
    {
        registerChild(this);
    }
}

// 构造函数(基类)
CheckDevice::CheckDevice(TIM_HandleTypeDef *htim)
{
	htim_ = htim;
}

// 注册子类实例
void CheckDevice::registerChild(CheckDevice *child)
{
    children_[childCount_++] = child;
}

// 初始化定时器
void CheckDevice::initTimer()
{
    HAL_TIM_Base_Start_IT(htim_);
}

// 更新所有子类标志位
void CheckDevice::timing_processing()
{
    for (int i = 0; i < childCount_; i++)
    {	
		children_[i]->count_tick++;
		if(children_[i]->count_tick % 10 == 0){
		   children_[i]->isActiveFlag = false;
		}
        children_[i]->onTimerEvent(); // 可选：直接调用处理函数
    }
}

// 定时器中断回调
extern "C" void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (CheckDevice::htim_ == htim)
    {
        CheckDevice::timing_processing();
    }
	
	
	
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM5) {
		HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}
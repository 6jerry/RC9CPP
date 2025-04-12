# CheckDevice 类使用文档

## 项目概述
`CheckDevice` 是一个用于管理设备定时器事件和状态更新的 C++ 类。它提供了一个统一的接口，用于处理多个设备的定时器事件，并允许子类通过继承实现自定义的定时器处理逻辑。

## 类结构
### 文件结构
- **Check_Device.h**：类定义和接口声明
- **Check_Device.cpp**：类实现和逻辑代码

### 类定义
```cpp
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
    uint32_t count_tick = 0; // 子类继承的计数器

protected:
    virtual void onTimerEvent(){} // 子类实现的定时事件处理
    
private:
    static CheckDevice *children_[MAX_CHILD_INSTANCES];
    static uint8_t childCount_;
};
```

## 成员变量
### 静态变量
- **`children_`**：存储所有子类实例的数组，最大支持 `MAX_CHILD_INSTANCES` 个实例。
- **`childCount_`**：当前已注册的子类实例数量。
- **`htim_`**：指向定时器句柄的指针，用于统一管理定时器。

### 实例变量
- **`isActiveFlag`**：布尔值，表示设备的活动状态。
- **`count_tick`**：计数器，用于记录定时器事件的触发次数。

## 成员函数
### 构造函数
- **`CheckDevice()`**：默认构造函数，用于创建 `CheckDevice` 实例。
- **`CheckDevice(TIM_HandleTypeDef *htim)`**：带参数的构造函数，用于初始化定时器。

### 静态函数
- **`registerChild(CheckDevice *child)`**：将子类实例注册到 `children_` 数组中。
- **`timing_processing()`**：定时器事件处理函数，遍历所有子类实例并调用它们的 `onTimerEvent()` 函数。
- **`initTimer()`**：初始化定时器，启动定时器中断。

### 虚函数
- **`onTimerEvent()`**：纯虚函数，子类必须实现该函数以处理定时器事件。

## 使用示例

### 1：创建子类并实现 `onTimerEvent()`(可选，isActiveFlag默认为定时刷新，需要在外设中断回调中重新置位)
```cpp
#include "Check_Device.h"

class MyDevice : public CheckDevice {
public:
    void onTimerEvent() override {
        // 在这里实现定时器事件的处理逻辑(可以不执行但是需要定义)
       
    }
    void handleReceiveData(uint8_t byte){
        isActiveFlag = true; //更新标志位，说明设备活跃
    }

};
```

### 示例 2：基类初始化和使用
```cpp
CheckDevice check_device(&htim7);

void test(){

		check_device.initTimer(); //用于启动定时器
		encoder.startUartReceiveIT();
		osKernelStart();
	}

```

## 注意事项
1. **定时器配置**：确保定时器已正确配置并启用中断。
2. **子类实现**：所有继承自 `CheckDevice` 的子类必须实现 `onTimerEvent()` 函数。
3. **实例数量限制**：最多支持 `MAX_CHILD_INSTANCES` 个子类实例。
4. **线程安全**：`timing_processing()` 函数在定时器中断中调用，确保线程安全。

## 常见问题解答
### Q1: 如何确保定时器事件的及时处理？
A: 确保定时器的中断优先级设置合理，并在 `onTimerEvent()` 中尽量减少耗时操作。

### Q2: 如何扩展 `CheckDevice` 的功能？
A: 通过继承 `CheckDevice` 并实现自定义的 `onTimerEvent()` 函数来扩展功能。

### Q3: 如何调试定时器事件？
A: 在 `onTimerEvent()` 中添加调试输出或使用断点进行调试。

## 参考资料
- STM32 HAL 库文档
- C++ 多态和虚函数使用指南
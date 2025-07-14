# Auxiliary Timer 使用手册

**版本：** v1.0  
**作者：** loopgad  
**日期：** 2025-07-12  

---

## 1. 概述
`Auxiliary Timer` 是一个基于 STM32 HAL 的轻量级 C++ 定时器框架，支持：

- **单一定时器资源**管理多个子任务  
- **微秒级 / 毫秒级** 计时接口  
- **面向对象** 的扩展方式，便于模块化开发  
- **最大 32 个子类实例**  

---

## 2. 文件结构

| 文件                | 作用                         |
|---------------------|------------------------------|
| `Auxiliary_Timer.h` | 类声明、宏定义、接口说明     |
| `Auxiliary_Timer.cpp` | 类实现、静态变量初始化、逻辑 |

---

## 3. 快速开始

### 3.1 硬件准备
- 在 CubeMX / CubeIDE 中开启定时器并配置好分频与计数值（每次触发时为1us）启用 **定时器中断**

### 3.2 代码集成

#### 3.2.1(1) 用cubemx重新生成工程后，将`main.c`属性修改为使用cpp编译
```c
extern "C" void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{

 /* USER CODE BEGIN Callback 0 */
   if (auxiliary_timer::htim_ == htim)
   {
       auxiliary_timer::timing_processing();
   }
 /* USER CODE END Callback 0 */
 if (htim->Instance == TIM5) {
		HAL_IncTick();
 }
 /* USER CODE BEGIN Callback 1 */

 /* USER CODE END Callback 1 */
}
```

3.2.1(2) 用cubemx重新生成工程后，把 `main.c` 中
```c
extern "C" void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{

 /* USER CODE BEGIN Callback 0 */
 /* USER CODE END Callback 0 */
 if (htim->Instance == TIM5) {
		HAL_IncTick();
 }
 /* USER CODE BEGIN Callback 1 */

 /* USER CODE END Callback 1 */
}
```
代码删除，解除`Auxiliary_Timer.cpp`中的HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)注释

#### 3.2.2 初始化基类
```cpp
// 在 setup启动文件
auxiliary_timer auxiliary_timer_(&htim2);   // 替换为你的 TIM_HandleTypeDef指针
auxiliary_timer.initTimer();      // 启动中断
```

---

## 4. 使用方式

### 4.1 创建自定义定时任务

#### 示例：LED 闪烁任务
```cpp
#include "Auxiliary_Timer.h"

class LedBlinker : public auxiliary_timer
{
public:
    void onTimerEvent() override
    {
        if (get_delta_ms() > 500.0f) {  // 每 500 ms 翻转一次
            HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
            set_delta_ms();             // 重置计时区间
        }
    }
};

// 全局实例（自动注册）
LedBlinker blinker;
```

### 4.2 微秒级计时示例
```cpp
class PulseMeasure : public auxiliary_timer
{
public:
    void onTimerEvent() override
    {
        set_delta_us();                    // 开始计时
        // ... 执行待测代码 ...
        uint64_t elapsed = get_delta_us(); // 获取耗时
    }
};
```

---

## 5. API 参考

| 成员                | 类型   | 说明 |
|---------------------|--------|------|
| `auxiliary_timer()` | 构造函数 | **子类** 使用，自动注册 |
| `auxiliary_timer(TIM_HandleTypeDef*)` | 构造函数 | **基类** 使用，绑定硬件定时器 |
| `static void initTimer()` | 方法 | 启动 HAL 定时器中断 |
| `static void timing_processing()` | 方法 | 在中断回调中调用，驱动所有子任务 |
| `void onTimerEvent()` | 虚函数 | **子类重写**，实现自定义逻辑 |
| `set_delta_ms()` / `get_delta_ms()` | 方法 | 毫秒级区间计时 |
| `set_delta_us()` / `get_delta_us()` | 方法 | 微秒级区间计时 |
| `count_tick_us` / `count_tick_ms` | 变量 | 累计计时值（只读） |

---

## 6. 注意事项

| 事项 | 说明 |
|------|------|
| **中断频率** | 推荐 1 kHz（1 ms），频率越高 CPU 占用越大 |
| **实例上限** | 最多 32 个子类实例（`MAX_CHILD_INSTANCES`） |
| **线程安全** | 所有回调运行于中断上下文，避免长时间阻塞 |
| **CubeMX 配置** | 定时器需勾选 **“Generate IRQ handler”** |
| **C++ 支持** | 确保编译器启用 **GNU++11** 或更高标准 |

---

## 7. 常见问题

| 问题 | 解决方案 |
|------|----------|
| `onTimerEvent()` 不触发 | 检查中断回调是否正确调用 `timing_processing()` |
| 计时误差大 | 确认定时器实际中断频率与配置一致 |
| 编译错误 “undefined reference to `vtable`” | 确保所有虚函数均已实现 |

---

> **Happy Coding!**
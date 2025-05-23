

# 陀螺仪传感器驱动与数据处理模块

## 概述
本项目包含了一系列用于陀螺仪传感器（如 Wit-Motion 传感器）的驱动和数据处理代码。这些代码提供了从传感器初始化、数据读取到数据处理的完整功能，适用于嵌入式系统（如 STM32）和上位机（如 PC）的开发。

## 文件结构
- **`gyro.c` 和 `gyro.h`**：
  - 提供了陀螺仪传感器的基本操作，包括初始化、数据更新和角度计算。
- **`GYRO_fuc.cpp` 和 `GYRO_fuc.h`**：
  - 提供了面向对象的接口，用于从传感器读取数据并处理角度信息。
- **`wit_c_sdk.c` 和 `wit_c_sdk.h`**：
  - 提供了与 Wit-Motion 传感器通信的底层库，支持多种通信协议（如 UART、I2C、CAN）。
- **`REG.h`**：
  - 定义了传感器寄存器地址和相关宏定义，用于读写传感器寄存器。

## 功能模块

### 1. 陀螺仪传感器初始化
#### 文件：`gyro.c` 和 `gyro.h`
- **功能**：
  - 初始化陀螺仪传感器，设置通信协议和地址。
  - 提供数据更新函数，用于读取传感器数据并计算角度。
- **使用方法**：
  ```c
  GYR_Init();  // 初始化传感器
  GYR_Updata();  // 更新数据并计算角度
  ```

### 2. 数据处理
#### 文件：`GYRO_fuc.cpp` 和 `GYRO_fuc.h`
- **功能**：
  - 提供了一个 C++ 类 `GYRO`，用于处理传感器数据。
  - 支持从传感器读取数据并计算角度（如 Yaw、Pitch、Roll）。
- **使用方法**：
  ```cpp
  GYRO gyro(&huartx);  // 创建 GYRO 对象，传入 UART 句柄
  gyro.Get_Data();  // 获取数据并计算角度
  float yaw_angle = gyro.Yaw_angle;  // 获取 Yaw 角度
  ```

### 3. 传感器通信库
#### 文件：`wit_c_sdk.c` 和 `wit_c_sdk.h`
- **功能**：
  - 提供了与 Wit-Motion 传感器通信的底层库。
  - 支持 UART、I2C 和 CAN 通信协议。
  - 提供了寄存器读写功能，用于配置传感器。
- **使用方法**：
  ```c
  WitInit(WIT_PROTOCOL_NORMAL, 0x50);  // 初始化传感器通信
  WitWriteReg(RATE, 0x01);  // 设置传感器输出速率
  WitReadReg(AX, 3);  // 读取加速度数据
  ```

### 4. 寄存器定义
#### 文件：`REG.h`
- **功能**：
  - 定义了传感器寄存器地址和相关宏定义。
  - 提供了寄存器地址的映射，便于读写操作。
- **使用方法**：
  ```c
  #include "REG.h"
  WitWriteReg(AXOFFSET, 0x1234);  // 设置加速度计 X 轴偏移
  ```

## 示例代码
以下是一个完整的示例代码，展示如何使用这些模块初始化传感器并读取数据。

```cpp
#include "gyro.h"
#include "GYRO_fuc.h"
#include "wit_c_sdk.h"

// 初始化传感器
void setup() {
    GYR_Init();  // 初始化陀螺仪
    WitInit(WIT_PROTOCOL_NORMAL, 0x50);  // 初始化通信协议
}

// 主循环
void loop() {
    GYRO gyro(&huartx);  // 创建 GYRO 对象
    gyro.Get_Data();  // 获取数据并计算角度

    float yaw_angle = gyro.Yaw_angle;  // 获取 Yaw 角度
    float pitch_angle = gyro.Pitch_angle;  // 获取 Pitch 角度
    float roll_angle = gyro.Roll_angle;  // 获取 Roll 角度

    printf("Yaw: %.2f, Pitch: %.2f, Roll: %.2f\n", yaw_angle, pitch_angle, roll_angle);

    delay(1000);  // 等待1秒
}
```

## 注意事项
1. **通信协议**：
   - 确保选择正确的通信协议（如 UART、I2C、CAN）。
   - 根据传感器型号和开发环境选择合适的协议。

2. **寄存器配置**：
   - 使用 `WitWriteReg` 和 `WitReadReg` 函数配置和读取寄存器。
   - 参考 `REG.h` 文件中的寄存器地址和定义。

3. **数据处理**：
   - 使用 `GYRO` 类处理传感器数据，获取角度信息。
   - 确保在调用 `Get_Data` 方法前完成传感器初始化。

## 支持的平台
- **STM32**：适用于 STM32 系列微控制器。
- **PC**：适用于上位机开发，通过 UART 或 USB 与传感器通信。

## 联系方式
- **官方网站**：[Wit-Motion 官方网站](http://wit-motion.cn/)
- **技术支持**：[Wit-Motion 技术支持](mailto:support@wit-motion.cn)

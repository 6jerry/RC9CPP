# LaserProcessor 使用方法

## 概述

`LaserProcessor` 是一个用于处理激光测距模块数据的 C++ 类。它提供了初始化命令的发送、数据的接收和处理、以及错误状态的跟踪功能。该类设计用于嵌入式系统，特别是 STM32 微控制器，结合 HAL 库使用。

## 文件结构

- **`laser_processor.h`**：定义了 `LaserProcessor` 类及其接口。
- **`laser_processor.cpp`**：实现了 `LaserProcessor` 类的功能。

## 主要功能

### 1. 初始化命令

`LaserProcessor` 类包含一组预定义的初始化命令，用于配置激光测距模块。这些命令包括：

- 开启激光
- 设置分辨率
- 设置测量频率
- 开始连续测量

### 2. 数据处理

类提供了 `ProcessByte` 方法，用于逐字节处理从激光模块接收到的数据。它支持：

- 数据校验
- ASCII 数据解析
- 移动平均滤波
- 距离变化率限制

### 3. 错误状态跟踪

每个初始化命令都有一个状态跟踪器，用于记录命令的发送状态，包括：

- 等待响应
- 成功
- 校验和错误
- 超时
- 设备错误

### 4. 初始化命令发送

`SendInitCommands` 函数用于发送初始化命令，并处理命令的重试逻辑。

## 使用方法

### 1. 初始化激光处理器

在主函数中初始化 `LaserProcessor` 对象，构造函数自动调用串口

```cpp
#include "laser_processor.h"

LaserProcessor laser(&huartx);


int main(void) {
    // 初始化硬件
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_USART1_UART_Init();

    // 初始化激光处理器
    SendInitCommands();

    while (1) {
        // 主循环
    }
}
```

### 2. 处理接收到的数据

参照原始serial_device继承的其他外设用法

## 类的结构

### `LaserProcessor` 类

```cpp
class LaserProcessor {
public:
    LaserProcessor();
    const Command* InitCommands() const;
    static constexpr int InitCommandCount() { return CMD_GROUP_SIZE; }
    float get_distance(uint8_t byte);
    CmdStatus GetCmdStatus(int index) const;

private:
    uint8_t rx_buffer_[32];
    uint16_t rx_length_;
    uint8_t resolution_;
    float filter_window_[FILTER_WINDOW_SIZE];
    uint8_t data_count_;
    uint8_t window_index_;
    float last_data_;
    struct {
        CmdStatus status;
        uint8_t retries;
        uint32_t sent_time;
    } cmd_tracker_[CMD_GROUP_SIZE];

    bool ValidatePacket(const uint8_t* data, uint16_t len);
    float ParseAsciiDistance(const uint8_t* start, uint16_t bytes);
    uint8_t CalculateChecksum(const uint8_t* data, uint16_t len);
    float ProcessByte(uint8_t byte);
    void ProcessCmdResponse(const uint8_t* data, uint8_t len);
};
```

## 错误处理

`LaserProcessor` 类提供了错误状态跟踪功能，可以通过 `GetCmdStatus` 方法查询每个命令的状态。

## 注意事项

- 确保激光测距模块的通信协议与代码中的命令一致。
- 在发送命令时，确保 UART 配置与模块的通信参数匹配。
- 在处理接收到的数据时，确保数据格式与模块的输出一致。

## 使用示例

**init_code**

```c
LaserProcessor::LaserProcessor(UART_HandleTypeDef *huart_) : 
    SerialDevice(huart_), rx_length_(0), resolution_(2), data_count_(0), 
    window_index_(0), last_data_(0){
    memset(filter_window_, 0, sizeof(filter_window_));
    memset(cmd_tracker_, 0, sizeof(cmd_tracker_));

    for(int i=0; i<CMD_GROUP_SIZE; i++) {
        int retry = 0;
        do {
            // 发送命令
            const auto& cmd = InitCommands()[i];
            HAL_UART_Transmit(huart_, cmd.data, cmd.length, HAL_MAX_DELAY);
        
            // 重置状态
            cmd_tracker_[i].sent_time = HAL_GetTick();
            cmd_tracker_[i].status = CMD_PENDING;
        
            // 等待响应（200ms超时）
            while((HAL_GetTick() - cmd_tracker_[i].sent_time) < 200) {
                if(GetCmdStatus(i) != CMD_PENDING) break;
                HAL_Delay(10);
            }
        
            // 处理超时
            if(GetCmdStatus(i) == CMD_PENDING) {
                cmd_tracker_[i].status = CMD_TIMEOUT;
            }
        
        } while(retry++ < 3 && 
              (laser.GetCmdStatus(i) == LaserProcessor::CMD_TIMEOUT ||
               laser.GetCmdStatus(i) == LaserProcessor::CMD_CHECKSUM_ERR));
    
        HAL_Delay(50); // 保持协议要求的时间间隔
    }
}
```

**recieve_code**

```c
void LaserProcessor::handleReceiveData(uint8_t byte){
    laser_distance = get_distance(byte);
}
//read the class public data **laser_distance**
```

## Notion

---

**波特率：9600bps**
串口初始化后调用构造函数
------------------------

串口接收使用请见[serialdevice](https://github.com/6jerry/RC9CPP/blob/shootercar/RC9CPP_API%E4%BD%BF%E7%94%A8%E6%89%8B%E5%86%8C/%E7%A1%AC%E4%BB%B6%E7%B1%BB/%E4%B8%B2%E5%8F%A3%E5%9F%BA%E7%B1%BBserial_device.md)

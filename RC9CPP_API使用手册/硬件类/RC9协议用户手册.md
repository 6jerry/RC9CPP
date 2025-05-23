# RC9协议用户手册

## 1. 协议概述

RC9协议是一种用于STM32微控制器的串行通信协议，支持通过UART或USB CDC（虚拟串口）进行通信。它提供了可靠的数据传输机制，支持帧校验，并采用发布-订阅模式方便多模块间的数据交换。

### 1.1 帧格式

RC9协议帧结构如下：

| 帧头(2字节) | ID(1字节) | 数据长度(1字节) | 数据载荷(最多64字节) | CRC校验(2字节) | 帧尾(2字节) |
| ----------- | --------- | --------------- | -------------------- | -------------- | ----------- |
| 0xFC 0xFB   | 帧ID      | 数据字节数      | 数据内容             | 校验码         | 0xFD 0xFE   |

- **帧头**：固定为 `0xFC 0xFB`，用于标识帧的开始
- **ID**：用于区分不同类型的数据帧，允许用户自定义
- **数据长度**：指定数据载荷的字节数
- **数据载荷**：实际传输的数据，最大支持64字节
- **CRC校验**：可选的CRC16校验码，用于确保数据完整性
- **帧尾**：固定为 `0xFD 0xFE`，用于标识帧的结束

## 2. 使用方法

### 2.1 创建RC9协议实例

首先需要创建一个RC9Protocol实例来处理通信：

```cpp
// 创建UART通信的RC9协议实例，启用CRC校验
RC9Protocol uart_protocol(uart, &huart1, true);

// 或创建虚拟串口(USB CDC)通信的RC9协议实例，不启用CRC校验
RC9Protocol cdc_protocol(cdc, nullptr, false);
```

参数说明：

- 第一个参数：通信类型（`uart`或`cdc`）
- 第二个参数：UART句柄（仅UART类型需要）
- 第三个参数：是否启用CRC校验

### 2.2 初始化发送队列

创建实例后，需要初始化发送队列以启用数据发送功能：

```cpp
uart_protocol.initQueue();
```

### 2.3 创建用户订阅者类

要接收和发送数据，需要创建一个继承自`RC9subscriber`的类：

```cpp
class MyDevice : public RC9subscriber
{
public:
    // 重写数据接收回调函数
    void DataReceivedCallback(const uint8_t *byteData, const float *floatData, 
                             uint8_t id, uint16_t byteCount) override
    {
        // 根据ID处理不同类型的数据
        switch(id)
        {
            case 0x01:
                // 处理字节数据
                handleCommand(byteData, byteCount);
                break;
            case 0x02:
                // 处理浮点数据
                handleSensorData(floatData, byteCount/4);
                break;
        }
    }
    
private:
    void handleCommand(const uint8_t *data, uint16_t length) {
        // 处理命令数据
    }
    
    void handleSensorData(const float *data, uint16_t count) {
        // 处理传感器数据
    }
};
```

### 2.4 建立订阅关系

创建用户类实例并与RC9协议实例建立订阅关系：

```cpp
MyDevice myDevice;
myDevice.addport(&uart_protocol);
```

通过调用`addport`方法，用户类会注册为协议实例的订阅者，同时获得向该协议实例发送数据的能力。

### 2.5 发送数据

RC9subscriber类提供了两种发送数据的方法：

#### 发送字节数据

```cpp
uint8_t cmd[3] = {0x01, 0x02, 0x03};
myDevice.sendByteData(0x01, cmd, 3);
```

参数说明：

- 第一个参数：帧ID
- 第二个参数：字节数据指针
- 第三个参数：数据长度

#### 发送浮点数据

```cpp
float values[2] = {3.14f, 2.71f};
myDevice.sendFloatData(0x02, values, 2);
```

参数说明：

- 第一个参数：帧ID
- 第二个参数：浮点数据指针
- 第三个参数：浮点数数量

### 2.6 数据处理

对于RC9Protocol实例，需要定期调用`process_data()`方法来处理发送队列中的数据。通常这应该在RTOS任务中进行：

```cpp
// 在任务处理函数中
void taskFunction(void* argument)
{
    while(1)
    {
        uart_protocol.process_data();
        osDelay(10);  // 适当延时
    }
}
```

## 3. 示例应用

### 3.1 温度传感器数据传输

```cpp
class TemperatureSensor : public RC9subscriber
{
public:
    void init(RC9Protocol *protocol)
    {
        addport(protocol);
    }
    
    void readAndSendTemperature()
    {
        float temp = readTemperature();
        sendFloatData(0x10, &temp, 1);
    }
    
    void DataReceivedCallback(const uint8_t *byteData, const float *floatData, 
                             uint8_t id, uint16_t byteCount) override
    {
        if (id == 0x11) {
            // 处理控制命令
            handleCommand(byteData, byteCount);
        }
    }
    
private:
    float readTemperature() {
        // 读取温度传感器
        return 25.5f;
    }
    
    void handleCommand(const uint8_t *data, uint16_t length) {
        // 处理控制命令
    }
};
```

### 3.2 主控程序

```cpp
// 创建协议实例
RC9Protocol uart_protocol(uart, &huart1, true);

// 创建传感器实例
TemperatureSensor tempSensor;

int main(void)
{
    // 初始化硬件...
    
    // 初始化通信
    uart_protocol.initQueue();
    uart_protocol.startUartReceiveIT();
    
    // 初始化传感器
    tempSensor.init(&uart_protocol);
    
    // 创建任务
    osThreadNew(communicationTask, NULL, NULL);
    
    // 启动调度器
    osKernelStart();
    
    while (1) {
        // 主循环...
    }
}

void communicationTask(void *argument)
{
    while (1) {
        // 定期读取并发送温度
        tempSensor.readAndSendTemperature();
        
        // 处理发送队列
        uart_protocol.process_data();
        
        osDelay(100);
    }
}
```

## 4. 协议限制

- 最大数据载荷：64字节
- 最多支持10个串口实例
- 每个协议实例最多支持5个订阅者
- 发送队列最大深度：5个数据帧

## 5. 注意事项

1. 使用前必须调用`initQueue()`初始化发送队列
2. 使用UART时应调用`startUartReceiveIT()`启动接收中断
3. 需要定期调用`process_data()`处理发送队列
4. CRC校验可提高通信可靠性，但会增加处理开销
5. 发送队列满时，`sendFloatData`和`sendByteData`会返回`false`

通过遵循本手册的指导，您可以轻松地在STM32应用中使用RC9协议实现可靠的数据通信。
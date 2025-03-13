![](G:\STM32_PROJECT\R1_NEW\R1_No2 - 改动\RC9CPP_API使用手册\硬件类\SerialDevice.png)

# SerialDevice 类库使用手册

## 1. 介绍

SerialDevice 是一个用于 STM32 平台的 C++ 串口通信基类库，旨在简化单片机与各种串口设备的交互。该类库设计了一套面向对象的架构，使得添加新的串口设备变得简单高效。支持标准 UART 接口和虚拟串口(CDC)两种模式。

## 2. 架构设计

### 2.1 设计思想

SerialDevice 采用了面向对象的设计模式，通过基类和派生类的组合，实现了高度的可扩展性：

- **基类 (SerialDevice)**: 提供串口通信的通用功能，包括初始化、数据接收中断等。
- **派生类**: 实现特定设备的协议解析和数据处理逻辑。

这种设计允许用户只需关注特定设备的协议解析，而不必重复编写低级的串口处理代码。

### 2.2 核心功能

- 自动管理多个串口设备实例
- 支持标准UART和虚拟串口(CDC)接口
- 提供简单的接口用于接收和处理串口数据
- 使用中断方式接收数据，提高系统响应性能

## 3. SerialDevice 基类详解

### 3.1 主要属性

```cpp
static SerialDevice *instances_[MAX_INSTANCES]; // 全局实例数组
static int instanceCount_;                      // 实例计数
static SerialDevice *cdc_instance;              // 虚拟串口实例
UART_HandleTypeDef *huart_;                     // UART句柄
uint8_t rxBuffer_[RX_BUFFER_SIZE];              // 接收缓冲区
uart_type type;                                 // 串口类型（标准UART或CDC）
```

### 3.2 主要方法

#### 构造函数

```cpp
SerialDevice(UART_HandleTypeDef *huart, uart_type type_ = uart);
```

- 参数

  :

  - `huart`: UART外设句柄指针
  - `type_`: 串口类型，默认为标准UART

#### 静态方法

```cpp
static void registerInstance(SerialDevice *instance);     // 注册标准UART实例
static void registerCDCInstance(SerialDevice *instance);  // 注册虚拟串口实例
```

#### 虚函数

```cpp
virtual void handleReceiveData(uint8_t byte) = 0;  // 纯虚函数，子类必须实现
```

#### 其他方法

```cpp
void startUartReceiveIT();  // 启动UART接收中断
```

## 4. 使用方法

### 4.1 创建新的设备类

要为特定设备创建串口通信类，需要从 SerialDevice 继承并实现必要的函数：

```cpp
class MyDevice : public SerialDevice {
public:
    // 构造函数
    MyDevice(UART_HandleTypeDef *huart) : SerialDevice(huart) {
        // 初始化代码
    }
    
    // 必须实现的数据处理函数
    void handleReceiveData(uint8_t byte) override {
        // 实现设备特定的数据处理逻辑
    }
    
private:
    // 设备特定属性和方法
};
```

### 4.2 实例化和使用

在应用程序中实例化和使用自定义设备类：

```cpp
// 声明设备实例
MyDevice myDevice(&huart1);

// 初始化UART和启动接收
void initDevices() {
    myDevice.startUartReceiveIT();
}

// 在主程序中使用
int main() {
    initDevices();
    
    while (1) {
        // 主循环代码
    }
}
```

## 5. 案例分析: HWT101CT 陀螺仪模块

以下通过分析 HWT101CT 陀螺仪模块的实现，展示如何使用 SerialDevice 基类开发具体设备驱动。

### 5.1 类定义

HWT101CT 类继承自 SerialDevice 和 imu 接口类，实现了陀螺仪的数据接收和解析功能。

```cpp
class HWT101CT : public SerialDevice, public imu {
    // ...实现细节
};
```

### 5.2 协议处理

HWT101CT 类通过状态机模式实现对陀螺仪协议的解析：

```cpp
enum RxState {
    WAITING_FOR_HEADER_1,
    WAITING_FOR_HEADER_2,
    // ...其他状态
};
```

### 5.3 数据处理

在 `handleReceiveData` 函数中实现了协议解析逻辑：

```cpp
void HWT101CT::handleReceiveData(uint8_t byte) {
    switch (rx_state) {
    case WAITING_FOR_HEADER_1:
        // 处理第一个帧头
        break;
    // ...其他状态处理
    }
}
```

## 6. 开发自定义设备的步骤

1. **继承 SerialDevice 基类**:

   ```cpp
   class MyDevice : public SerialDevice { ... };
   ```

2. **实现协议解析状态机**:

   ```cpp
   enum RxState { ... };
   ```

3. **重写 handleReceiveData 方法**:

   ```cpp
   void handleReceiveData(uint8_t byte) override { ... }
   ```

4. **实现数据处理逻辑**:

   ```cpp
   void processDecodedData(...) { ... }
   ```

5. **初始化并启动接收**:

   ```cpp
   MyDevice device(&huart1);
   device.startUartReceiveIT();
   ```

## 7. 注意事项

1. **最大实例数**: 系统最多支持 10 个串口实例，可通过修改 `MAX_INSTANCES` 进行调整。
2. **接收缓冲区大小**: 默认的接收缓冲区大小为 1 字节，适用于字节流处理模式。
3. **中断处理**: 系统使用全局 HAL 库回调函数处理接收中断，确保在使用前已正确配置 UART 外设。
4. **虚拟串口**: 使用虚拟串口时，需要将类型设置为 `cdc`，并确保 USB CDC 功能已正确配置。

## 8. 高级应用

### 8.1 多设备管理

SerialDevice 类库支持同时管理多个串口设备，只需创建不同的实例并分配不同的 UART 句柄：

```cpp
DeviceA deviceA(&huart1);
DeviceB deviceB(&huart2);
DeviceC deviceC(&huart3);
```

### 8.2 虚拟串口(CDC)支持

若要使用虚拟串口，实例化时指定类型为 `cdc`：

```cpp
MyDevice myDevice(nullptr, cdc);  // 注意：CDC 模式下 huart 参数为 nullptr
```

## 9. 总结

SerialDevice 类库提供了一个简洁而强大的框架，用于开发 STM32 平台的串口设备驱动。通过继承基类并实现特定协议解析逻辑，可以快速开发各种串口设备的驱动程序。该框架的设计理念是将通用的串口通信功能抽象到基类中，让开发者专注于设备特定的协议实现，提高代码复用性和开发效率。
# 错误检测模块使用手册

## 概述

本模块提供了一套灵活的错误检测机制，用于监控和管理系统中各种硬件设备的运行状态。通过统一的错误码和设备标识符，能够方便地识别和处理设备故障。

## 功能特性

- **设备错误检测**：支持多种硬件设备的错误检测，包括电机、光电门传感器、ROS 通信模块等。
- **错误码管理**：使用统一的错误码枚举类型，便于识别和处理各种错误情况。
- **实例注册管理**：通过全局数组管理所有注册的 `error_check` 实例，支持最多 16 个实例的注册。
- **错误信息发送**：当检测到错误时，能够通过指定通信 ID 发送错误信息。
- **超时检测**：支持超时检测机制，当操作超时时，能够自动触发错误处理。

## 安装与配置

### 环境要求

- 支持 `TaskManager.h` 和 `RC9Protocol.h` 的开发环境
- 文件优化等级使用-o2

### 安装步骤

1. 将 `err_code.cpp` 和 `err_code.h` 文件添加到项目中。
2. 确保项目能够正确引用 `TaskManager.h` 和 `RC9Protocol.h` 头文件。

## 使用指南

### 1. 定义设备错误检测类

首先，需要为每个具体的硬件设备定义一个继承自 `error_check` 的错误检测类。例如，为电机定义一个错误检测类：
必须实现虚函数，在类声明时为err_id进行赋值，同时要在设备活跃时对timeout_flag标志位进行置位（置位为false）

```cpp
class MotorErrorCheck : public error_check {
public:
    err_code check_error() override {
        // 实现电机的错误检测逻辑
        // 如果检测到错误，返回相应的错误码
        // 如果检测正常，返回 ERR_CODE_WORK_SUCCESS
    }
};


	err_id = ERR_DECVICE_MOTOR;
}
```

### 2. 注册错误检测实例

在主程序中创建错误检测类的子类的实例，它们会自动注册到全局管理数组中：

```cpp
int main() {
    MotorErrorCheck motorErrorCheck; // 创建电机错误检测
    // ...
    return 0;
}
```

### 3. 处理错误信息

需要初始化erroe_checker对象，并添加串口，他的process_data中将会轮询所有的error_check子类，如果出现错误则通过串口发送错误信息。

```cpp
int main() {

        error_checker errorChecker;
        errorChecker.addport(&err_msg_port);
        task_core.registerTask(4, &errorChecker);
    return 0;
}
```

### 4. 发送错误信息

注意要将errorChecker注册到任务
当 `process_data` 方法检测到错误时，会自动调用 `sendByteData` 函数发送错误信息。确保在项目中实现了该函数，以便能够正确发送数据：

```cpp
task_core.registerTask(4, &errorChecker);
```

## 错误码说明

| 错误码                        | 说明         |
| :---------------------------- | :----------- |
| `ERR_CODE_WORK_SUCCESS`     | 操作成功完成 |
| `ERR_CODE_NOT_INITIALIZED`  | 模块未初始化 |
| `ERR_CODE_FAIL_INITIALIZED` | 初始化失败   |
| `ERR_CODE_TIMEOUT`          | 操作超时     |
| `ERR_CODE_CONNECT_FAIL`     | 连接失败     |
| `ERR_CODE_DISCONNECT`       | 设备断开连接 |

## 设备标识符说明

| 设备标识符                    | 说明         |
| :---------------------------- | :----------- |
| `ERR_DEVICE_NONE`           | 无设备       |
| `ERR_DEVICE_MOTOR`          | 电机         |
| `ERR_DEVICE_PHOTOGATE`      | 光电门传感器 |
| `ERR_DEVICE_ROS`            | ROS 通信模块 |
| `ERR_DEVICE_POSITION`       | 定位模块     |
| `ERR_DEVICE_LASER`          | 激光测距模块 |
| `ERR_DEVICE_ULTRASONIC`     | 超声波传感器 |
| `ERR_DEVICE_SOLENOID_VALVE` | 电磁阀       |

## 注意事项

- 确保每个设备的错误检测类都正确实现了 `check_error` 和 `set_device_id` 方法。
- 在注册实例时，注意不要超过 `MAX_INSTANCES` 限制。
- 在发送错误信息时，确保 `sendByteData` 函数能够正确实现并发送数据。

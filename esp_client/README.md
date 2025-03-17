# ESP32 Xbox WIFI 控制器

## 概述

本工程实现了一个基于 ESP32 的 Xbox 控制器无线通信系统。ESP32 通过蓝牙与 Xbox 控制器连接，并通过 Wi-Fi 将控制器的输入数据发送到指定的 UDP 服务器。该工程适用于需要通过 Xbox 控制器远程控制设备的场景，例如机器人控制、无人机控制等。

## 硬件要求

- ESP32 开发板
- Xbox Series X/S 控制器
- Wi-Fi 网络

## 软件依赖

- Arduino IDE
- ESP32 Arduino Core
- NimBLE-Arduino 库（用于蓝牙通信）
- XboxSeriesXControllerESP32_asukiaaa 库（用于 Xbox 控制器通信）

## 工程结构

- **crc_util.h/crc_util.c**: 提供了 CRC16 和 CRC8 校验码的计算功能，用于数据校验。
- **main.cpp**: 主程序文件，负责初始化 Wi-Fi 和蓝牙连接，并处理 Xbox 控制器的输入数据。
- **RC9Protocol.hpp/RC9Protocol.cpp**: 定义了数据帧的打包和发送逻辑，支持通过 UDP 发送数据。
- **XboxControllerNotificationParser.h/XboxControllerNotificationParser.cpp**: 解析 Xbox 控制器的输入数据，并将其转换为可用的格式。
- **XboxSeriesXControllerESP32_asukiaaa.hpp**: 提供了与 Xbox 控制器通信的核心功能。
- **XboxSeriesXHIDReportBuilder_asukiaaa.hpp**: 定义了 Xbox 控制器的 HID 报告结构。

## 使用方法

1. **配置 Wi-Fi**: 在 `main.cpp` 中，修改 `WIFI_SSID` 和 `WIFI_PASS` 为你的 Wi-Fi 网络的 SSID 和密码。

   ```cpp
   #define WIFI_SSID "ESP32_AP"
   #define WIFI_PASS "12345678"
   ```

2. **配置 UDP 服务器**: 修改 `SERVER_IP` 和 `UDP_PORT` 为你的 UDP 服务器的 IP 地址和端口。

   ```cpp
   #define SERVER_IP "192.168.4.1"
   #define UDP_PORT 3333
   ```

3. **配置 Xbox 控制器蓝牙地址**: 在 `main.cpp` 中，将 `xboxController` 的蓝牙地址替换为你的 Xbox 控制器的蓝牙地址。

   ```cpp
   XboxSeriesXControllerESP32_asukiaaa::Core xboxController("0c:35:26:56:a0:22");
   ```

4. **编译并上传代码**: 使用 Arduino IDE 编译并上传代码到 ESP32 开发板。

5. **运行**: 启动 ESP32 后，它将尝试连接到 Xbox 控制器，并通过 Wi-Fi 将控制器的输入数据发送到指定的 UDP 服务器。

## 数据帧格式

数据帧的格式定义在 `RC9Protocol.hpp` 中，包含以下字段：

- **帧头**: `0xFC 0xFB`
- **帧 ID**: 1 字节
- **数据长度**: 1 字节
- **数据载荷**: 最多 64 字节
- **CRC16 校验码**: 2 字节
- **帧尾**: `0xFD 0xFE`

## 数据解析

Xbox 控制器的输入数据通过 `XboxControllerNotificationParser` 类进行解析，解析后的数据包括：

- 按钮状态（A、B、X、Y、LB、RB、LS、RS、Select、Start、Xbox、Share）
- 方向键状态（上、下、左、右）
- 左右摇杆的水平和垂直位置
- 左右触发器的值

## 注意事项

- 确保 Xbox 控制器已与 ESP32 配对。
- 确保 ESP32 和 UDP 服务器在同一网络中。
- 如果连接失败，ESP32 会自动重启并重新尝试连接。

## 参考

- [NimBLE-Arduino](https://github.com/h2zero/NimBLE-Arduino)
- [XboxSeriesXControllerESP32_asukiaaa](https://github.com/asukiaaa/XboxSeriesXControllerESP32)

## 许可证

本工程基于 MIT 许可证开源。
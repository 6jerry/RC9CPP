# ESP32C6 AP Server 项目说明

## 项目概述
这是一个基于ESP32-C6的WiFi AP服务器项目，提供UDP广播通信和TCP设备间消息转发功能。

## 主要功能
- 创建WiFi热点 (SSID: ESP32C6_AP)
- UDP服务器监听端口3333，支持消息广播
- 管理最多4个连接设备的状态信息
- 设备MAC地址和连接状态跟踪

## 硬件要求
- ESP32-C6开发板
- 支持802.11b/g/n/ax协议

## 软件配置
- 开发框架：ESP-IDF v5.4.0
- 主要组件：
  - lwIP网络协议栈
  - FreeRTOS实时操作系统
  - WiFi驱动

## 关键设置
```c
// WiFi热点配置
#define SSID "ESP32C6_AP"
#define PASSWORD "123456789"
#define MAX_CLIENTS 5

// 网络端口
#define PORT 3333

// 设备数量
#define NUM_DEVICES 4
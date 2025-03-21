#ifndef UDP_SERVER_H
#define UDP_SERVER_H

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <lwip/sockets.h>
#include <stdbool.h>
#include "esp_wifi.h"

void udp_server_start(void);

typedef struct {
    uint8_t mac[6];  // 客户端MAC地址
    int id;          // 设备ID
    struct sockaddr_in addr; // 客户端地址
    bool active;     // 是否活跃
} DeviceInfo;

// 初始化设备信息
void init_devices();

// 更新设备IP和端口
void update_device_info(const uint8_t *mac, struct sockaddr_in *addr);

// 通过MAC地址查找设备
DeviceInfo* find_device_by_mac(const uint8_t *mac);

// 预设的设备信息
extern DeviceInfo devices[4];

#endif
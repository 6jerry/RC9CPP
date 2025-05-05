#ifndef DEVICE_INFO_H
#define DEVICE_INFO_H

#include <lwip/sockets.h>
#include <stdbool.h>
#include "esp_wifi.h"
#include "esp_log.h"  // 添加ESP日志头文件

#define NUM_DEVICES 4

// 移除TAG定义

typedef struct {
    uint8_t mac[6];  // 客户端MAC地址
    int id;          // 设备ID
    struct sockaddr_in addr; // 客户端地址
    bool active;     // 是否活跃
} DeviceInfo;

extern DeviceInfo devices[NUM_DEVICES];

bool is_ip_already_assigned(struct sockaddr_in *addr);

void init_devices();
void update_device_info(const uint8_t *mac, struct sockaddr_in *addr);

#endif
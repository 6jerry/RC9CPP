#include "udp_server.h"
#include <stdio.h>
#include <string.h>
#include "esp_log.h"
#include "lwip/err.h"
#include "lwip/sys.h"
#include "lwip/sockets.h"

#define PORT 3333
#define NUM_DEVICES 4

static const char *TAG = "UDP_SERVER";

DeviceInfo devices[NUM_DEVICES];

// 初始化设备信息
void init_devices() {
    // 示例MAC地址和ID，您可以根据需要修改
    uint8_t mac1[6] = {0x00, 0x11, 0x22, 0x33, 0x44, 0x99};
    uint8_t mac2[6] = {0x00, 0x11, 0x22, 0x33, 0x44, 0x66};
    uint8_t mac3[6] = {0x00, 0x11, 0x22, 0x33, 0x44, 0x77};
    uint8_t mac4[6] = {0x00, 0x11, 0x22, 0x33, 0x44, 0x88};

    memcpy(devices[0].mac, mac1, 6);
    devices[0].id = 1;
    devices[0].active = false;

    memcpy(devices[1].mac, mac2, 6);
    devices[1].id = 2;
    devices[1].active = false;

    memcpy(devices[2].mac, mac3, 6);
    devices[2].id = 3;
    devices[2].active = false;

    memcpy(devices[3].mac, mac4, 6);
    devices[3].id = 4;
    devices[3].active = false;
}

// 更新设备信息
void update_device_info(const uint8_t *mac, struct sockaddr_in *addr) {
    for (int i = 0; i < NUM_DEVICES; i++) {
        if (memcmp(devices[i].mac, mac, 6) == 0) {
            devices[i].addr = *addr;
            devices[i].active = true;
            ESP_LOGI(TAG, "Device updated: ID=%d, IP=%s, Port=%d",
                    devices[i].id, inet_ntoa(addr->sin_addr), 
                    ntohs(addr->sin_port));
            return;
        }
    }
    ESP_LOGE(TAG, "Unknown device with MAC: %02x:%02x:%02x:%02x:%02x:%02x",
            mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
}




static void udp_server_task(void *pvParameters) {
    int addr_family = AF_INET;
    int ip_protocol = IPPROTO_IP;
    struct sockaddr_in dest_addr;

    dest_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    dest_addr.sin_family = AF_INET;
    dest_addr.sin_port = htons(PORT);

    int sock = socket(addr_family, SOCK_DGRAM, ip_protocol);
    if (sock < 0) {
        ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
        vTaskDelete(NULL);
        return;
    }

    int err = bind(sock, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
    if (err != 0) {
        ESP_LOGE(TAG, "Socket unable to bind: errno %d", errno);
        close(sock);
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI(TAG, "Socket listening on port %d", PORT);

    while (1) {
        struct sockaddr_in client_addr;
        socklen_t client_addr_len = sizeof(client_addr);
        char buffer[128];
        int len = recvfrom(sock, buffer, sizeof(buffer) - 1, 0, 
                         (struct sockaddr *)&client_addr, &client_addr_len);
        if (len > 0) {
            buffer[len] = '\0';
            
            // 获取客户端MAC地址
            uint8_t mac[6];
            // 这里需要添加获取MAC地址的代码，具体实现取决于您的网络配置
            
            // 更新设备信息
            update_device_info(mac, &client_addr);
            
            ESP_LOGI(TAG, "Received: %s", buffer);

            // 解析目标设备ID
            char *comma = strchr(buffer, ',');
            if (comma) {
                *comma = '\0';
                int target_id = atoi(buffer);
                char *message = comma + 1;

                // 查找目标设备
                if (target_id > 0 && target_id <= NUM_DEVICES) {
                    struct sockaddr_in target_addr = devices[target_id - 1].addr;
                    if (target_addr.sin_addr.s_addr != 0) {
                        // 直接使用recvfrom获取的端口号
                        target_addr.sin_port = client_addr.sin_port;
                        
                        // 转发消息
                        int send_len = sendto(sock, message, strlen(message), 0,
                                            (struct sockaddr *)&target_addr, sizeof(target_addr));
    

                        if (send_len < 0) {
                            ESP_LOGE(TAG, "Failed to send to device %d", target_id);
                        } else {
                            ESP_LOGI(TAG, "Forwarded message to device %d at port %d", 
                                    target_id, ntohs(client_addr.sin_port));
                        }
                    } else {
                        ESP_LOGE(TAG, "Device %d address not available", target_id);
                    }
                } else {
                    ESP_LOGE(TAG, "Invalid target device ID: %d", target_id);
                }
            } else {
                ESP_LOGE(TAG, "Invalid message format");
            }
        }
    }

    close(sock);
    vTaskDelete(NULL);
}


void udp_server_start(void) {
    init_devices();  // 初始化设备信息
    xTaskCreate(udp_server_task, "udp_server", 4096, NULL, 5, NULL);
}
#include "udp_server.h"
#include <stdio.h>
#include <string.h>
#include "esp_log.h"
#include "lwip/err.h"
#include "lwip/sys.h"
#include "lwip/sockets.h"
#include "device_info.h"

#define PORT 3333
#define NUM_DEVICES 4

static const char *UDP_TAG = "UDP_SERVER";  // 修改为UDP_TAG

//哪里使用哪里定义
// DeviceInfo devices[NUM_DEVICES];


static void udp_server_task(void *pvParameters) {
    int addr_family = AF_INET;
    int ip_protocol = IPPROTO_IP;
    struct sockaddr_in dest_addr;

    dest_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    dest_addr.sin_family = AF_INET;
    dest_addr.sin_port = htons(PORT);

    int sock = socket(addr_family, SOCK_DGRAM, ip_protocol);
    if (sock < 0) {
        ESP_LOGE(UDP_TAG, "Unable to create socket: errno %d", errno);
        vTaskDelete(NULL);
        return;
    }

    int err = bind(sock, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
    if (err != 0) {
        ESP_LOGE(UDP_TAG, "Socket unable to bind: errno %d", errno);
        close(sock);
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI(UDP_TAG, "Socket listening on port %d", PORT);

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
            // update_device_info(mac, &client_addr);
            
            ESP_LOGI(UDP_TAG, "Received: %s", buffer);

            // // 解析目标设备ID
            // char *comma = strchr(buffer, ',');
            // if (comma) {
            //     *comma = '\0';
            //     int target_id = atoi(buffer);
            //     char *message = comma + 1;

            //     // 查找目标设备
            //     if (target_id > 0 && target_id <= NUM_DEVICES) {
            //         struct sockaddr_in target_addr = devices[target_id - 1].addr;
            //         if (target_addr.sin_addr.s_addr != 0) {
            //             // 直接使用recvfrom获取的端口号
            //             target_addr.sin_port = client_addr.sin_port;
                        
            //             // 转发消息到目标设备
            //             int send_len = sendto(sock, message, strlen(message), 0,
            //                                   (struct sockaddr *)&target_addr, sizeof(target_addr));

            //             if (send_len < 0) {
            //                 ESP_LOGE(UDP_TAG, "Failed to send to device %d", target_id);
            //             } else {
            //                 ESP_LOGI(UDP_TAG, "Forwarded message to device %d at port %d", 
            //                          target_id, ntohs(client_addr.sin_port));
            //             }
            //         } else {
            //             ESP_LOGE(UDP_TAG, "Device %d address not available", target_id);
            //         }
            //     } else {
            //         ESP_LOGE(UDP_TAG, "Invalid target device ID: %d", target_id);
            //     }
            // } else {
            //     ESP_LOGE(UDP_TAG, "Invalid message format");
            // }
        
            // 广播消息到所有设备
            struct sockaddr_in broadcast_addr;
            broadcast_addr.sin_family = AF_INET;
            broadcast_addr.sin_port = htons(PORT);
            broadcast_addr.sin_addr.s_addr = htonl(INADDR_BROADCAST);

            int broadcast_len = sendto(sock, buffer, len, 0,
                                     (struct sockaddr *)&broadcast_addr, sizeof(broadcast_addr));
            if (broadcast_len < 0) {
                ESP_LOGE(UDP_TAG, "Failed to broadcast message");
            } else {
                ESP_LOGI(UDP_TAG, "Broadcasted message to all devices");
            }
        } else {
            ESP_LOGE(UDP_TAG, "Invalid message format");
        }
    }

    close(sock);
    vTaskDelete(NULL);
}



static void udp_server_task_test(void *pvParameters) {
    int addr_family = AF_INET;
    int ip_protocol = IPPROTO_IP;
    struct sockaddr_in dest_addr;

    dest_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    dest_addr.sin_family = AF_INET;
    dest_addr.sin_port = htons(PORT);

    int sock = socket(addr_family, SOCK_DGRAM, ip_protocol);
    if (sock < 0) {
        ESP_LOGE(UDP_TAG, "Unable to create socket: errno %d", errno);
        vTaskDelete(NULL);
        return;
    }

    int err = bind(sock, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
    if (err != 0) {
        ESP_LOGE(UDP_TAG, "Socket unable to bind: errno %d", errno);
        close(sock);
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI(UDP_TAG, "Socket listening on port %d", PORT);

    // 主动发送的字符串
    char message_to_send[64] = "Hello, device!";

    while (1) {
        // 遍历所有设备，向每个已连接的设备发送消息
        for (int i = 0; i < NUM_DEVICES; i++) {
            if (devices[i].active) {
                int send_len = sendto(sock, message_to_send, strlen(message_to_send), 0,
                                      (struct sockaddr *)&devices[i].addr, sizeof(devices[i].addr));
                if (send_len < 0) {
                    ESP_LOGE(UDP_TAG, "Failed to send message to device %d", devices[i].id);
                } else {
                    ESP_LOGI(UDP_TAG, "Sent message to device %d at IP %s, Port %d",
                             devices[i].id, inet_ntoa(devices[i].addr.sin_addr), ntohs(devices[i].addr.sin_port));
                }
            }
        }

        // 等待一段时间后再次发送，避免过快发送
        vTaskDelay(pdMS_TO_TICKS(1000)); // 每秒发送一次
    }

    close(sock);
    vTaskDelete(NULL);
}

void udp_server_start(void) {
    init_devices();  // 初始化设备信息
    xTaskCreate(udp_server_task, "udp_server", 8192, NULL, 5, NULL);
}
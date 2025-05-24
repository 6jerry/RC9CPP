#include "tcp_server.h"
#include <stdio.h>
#include <string.h>
#include "esp_log.h"
#include "lwip/err.h"
#include "lwip/sys.h"
#include "lwip/sockets.h"
#include "device_info.h"

static const char *TCP_TAG = "TCP_SERVER";  // 修改为TCP_TAG

DeviceInfo devices[NUM_DEVICES];

static void client_handler_task(void *pvParameters)
{
    int client_sock = *(int *)pvParameters;
    char buffer[128];
    int len;

    while ((len = recv(client_sock, buffer, sizeof(buffer) - 1, 0)) > 0) {
        buffer[len] = '\0';
        
        // 解析目标设备ID
        char *comma = strchr(buffer, ',');
        if (comma) {
            *comma = '\0';
            int target_id = atoi(buffer);
            char *message = comma + 1;

            // 查找目标设备
            if (target_id > 0 && target_id <= NUM_DEVICES) {
                if (devices[target_id - 1].active) {
                    // 转发消息到目标设备
                    int send_len = send(devices[target_id - 1].addr.sin_port, 
                                      message, strlen(message), 0);
                    if (send_len < 0) {
                        ESP_LOGE(TCP_TAG, "Failed to send to device %d", target_id);
                    } else {
                        ESP_LOGI(TCP_TAG, "Forwarded message to device %d", target_id);
                    }
                } else {
                    ESP_LOGE(TCP_TAG, "Device %d is not active", target_id);
                }
            } else {
                ESP_LOGE(TCP_TAG, "Invalid target device ID: %d", target_id);
            }

        //     // 广播消息到所有设备
        //     struct sockaddr_in broadcast_addr;
        //     broadcast_addr.sin_family = AF_INET;
        //     broadcast_addr.sin_port = htons(PORT);
        //     broadcast_addr.sin_addr.s_addr = htonl(INADDR_BROADCAST);

        //     int broadcast_len = sendto(client_sock, message, strlen(message), 0,
        //                              (struct sockaddr *)&broadcast_addr, sizeof(broadcast_addr));
        //     if (broadcast_len < 0) {
        //         ESP_LOGE(TCP_TAG, "Failed to broadcast message");
        //     } else {
        //         ESP_LOGI(TCP_TAG, "Broadcasted message to all devices");
        //     }
        // } else {
        //     ESP_LOGE(TCP_TAG, "Invalid message format");
        // }
        }
    }

    if (len < 0) {
        ESP_LOGE(TCP_TAG, "Error occurred during receiving: errno %d", errno);
    }

    ESP_LOGI(TCP_TAG, "Client disconnected");
    close(client_sock);
    vTaskDelete(NULL);
}

static void tcp_server_task(void *pvParameters)
{
    int addr_family = AF_INET;
    int ip_protocol = IPPROTO_IP;
    struct sockaddr_in dest_addr;

    dest_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    dest_addr.sin_family = AF_INET;
    dest_addr.sin_port = htons(PORT);

    int listen_sock = socket(addr_family, SOCK_STREAM, ip_protocol);
    if (listen_sock < 0) {
        ESP_LOGE(TCP_TAG, "Unable to create socket: errno %d", errno);
        vTaskDelete(NULL);
        return;
    }

    int err = bind(listen_sock, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
    if (err != 0) {
        ESP_LOGE(TCP_TAG, "Socket unable to bind: errno %d", errno);
        close(listen_sock);
        vTaskDelete(NULL);
        return;
    }

    err = listen(listen_sock, MAX_CLIENTS);
    if (err != 0) {
        ESP_LOGE(TCP_TAG, "Socket unable to listen: errno %d", errno);
        close(listen_sock);
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI(TCP_TAG, "Socket listening on port %d", PORT);

    while (1) {
        struct sockaddr_in client_addr;
        socklen_t client_addr_len = sizeof(client_addr);
        int client_sock = accept(listen_sock, (struct sockaddr *)&client_addr, &client_addr_len);
        if (client_sock < 0) {
            ESP_LOGE(TCP_TAG, "Unable to accept connection: errno %d", errno);
            continue;
        }

        uint8_t mac[6];
        esp_wifi_get_mac(WIFI_IF_AP, mac);
        update_device_info(mac, &client_addr);

        xTaskCreate(client_handler_task, "client_handler", 4096, (void *)&client_sock, 5, NULL);
    }

    close(listen_sock);
    vTaskDelete(NULL);
}

void tcp_server_start(void) {
    init_devices();
    xTaskCreate(tcp_server_task, "tcp_server", 8192, NULL, 5, NULL);
}
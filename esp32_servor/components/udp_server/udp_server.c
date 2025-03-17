#include "udp_server.h"
#include "esp_log.h"
#include "lwip/sockets.h"
#include "driver/uart.h"

static const char *TAG = "UDP_SERVER";
#define PORT 3333            // UDP服务器监听端口

void udp_server_task(void *pvParameters)
{
    char rx_buffer[128];     // 接收数据缓冲区
    int addr_family = AF_INET; // 使用IPv4地址族
    int ip_protocol = IPPROTO_IP; // 使用IP协议

    // 创建UDP socket
    int sock = socket(addr_family, SOCK_DGRAM, ip_protocol);
    if (sock < 0) {
        ESP_LOGE(TAG, "Unable to create socket");
        vTaskDelete(NULL);
        return;
    }

    // 配置服务器地址信息
    struct sockaddr_in server_addr;
    server_addr.sin_family = AF_INET;
    server_addr.sin_addr.s_addr = htonl(INADDR_ANY); // 监听所有网络接口
    server_addr.sin_port = htons(PORT); // 设置监听端口

    // 绑定socket到指定地址和端口
    if (bind(sock, (struct sockaddr *)&server_addr, sizeof(server_addr)) != 0) {
        ESP_LOGE(TAG, "Socket bind failed");
        close(sock);
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI(TAG, "UDP server started on port %d", PORT);

    while (1) {
        struct sockaddr_in client_addr;
        socklen_t client_len = sizeof(client_addr);
        
        // 接收客户端数据
        int len = recvfrom(sock, rx_buffer, sizeof(rx_buffer), 0,
                         (struct sockaddr *)&client_addr, &client_len);
        if (len < 0) {
            ESP_LOGE(TAG, "Error occurred during receiving");
            continue;
        }

        // 直接转发接收到的数据，不添加结束符
        uart_write_bytes(UART_NUM_1, rx_buffer, len);
        
        // 打印客户端发送的信息
        ESP_LOGI(TAG, "Received data from client: %.*s", len, rx_buffer);
    }

    // 关闭socket
    close(sock);
    vTaskDelete(NULL);
}
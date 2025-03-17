#include "tcp_server.h"
#include "esp_log.h"
#include "lwip/sockets.h"
#include "driver/uart.h"  // 添加uart.h头文件

static const char *TAG = "TCP_SERVER";
//WIFI_AP_DEF with IP: 192.168.4.1
#define PORT 3333            // TCP服务器监听端口
#define KEEPALIVE_IDLE 5     // TCP keepalive空闲时间(秒)
#define KEEPALIVE_INTERVAL 5 // TCP keepalive探测间隔(秒)
#define KEEPALIVE_COUNT 3    // TCP keepalive探测次数

void tcp_server_task(void *pvParameters)
{
    char rx_buffer[128];     // 接收数据缓冲区
    int addr_family = AF_INET; // 使用IPv4地址族
    int ip_protocol = IPPROTO_IP; // 使用IP协议

    // 配置服务器地址信息
    struct sockaddr_in server_addr;
    server_addr.sin_family = AF_INET;
    server_addr.sin_addr.s_addr = htonl(INADDR_ANY); // 监听所有网络接口
    server_addr.sin_port = htons(PORT); // 设置监听端口

    // 创建TCP socket
    int listen_sock = socket(addr_family, SOCK_STREAM, ip_protocol);
    if (listen_sock < 0) {
        ESP_LOGE(TAG, "Unable to create socket");
        vTaskDelete(NULL);
        return;
    }

    // 设置socket选项：允许地址重用
    int opt = 1;
    setsockopt(listen_sock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    // 绑定socket到指定地址和端口
    if (bind(listen_sock, (struct sockaddr *)&server_addr, sizeof(server_addr)) != 0) {
        ESP_LOGE(TAG, "Socket bind failed");
        close(listen_sock);
        vTaskDelete(NULL);
        return;
    }

    // 开始监听连接，最大连接数为1
    if (listen(listen_sock, 1) < 0) {
        ESP_LOGE(TAG, "Error listening on socket");
        close(listen_sock);
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI(TAG, "TCP server started on port %d", PORT);

    while (1) {
        struct sockaddr_in client_addr;
        socklen_t client_len = sizeof(client_addr);
        // 接受客户端连接
        int client_sock = accept(listen_sock, (struct sockaddr *)&client_addr, &client_len);
        if (client_sock < 0) {
            ESP_LOGE(TAG, "Unable to accept connection");
            continue;
        }

        ESP_LOGI(TAG, "Client connected");

        // 设置TCP keepalive选项
        int keepalive_idle = KEEPALIVE_IDLE;
        int keepalive_interval = KEEPALIVE_INTERVAL;
        int keepalive_count = KEEPALIVE_COUNT;
        setsockopt(client_sock, IPPROTO_TCP, TCP_KEEPIDLE, &keepalive_idle, sizeof(keepalive_idle));
        setsockopt(client_sock, IPPROTO_TCP, TCP_KEEPINTVL, &keepalive_interval, sizeof(keepalive_interval));
        setsockopt(client_sock, IPPROTO_TCP, TCP_KEEPCNT, &keepalive_count, sizeof(keepalive_count));

        // 处理客户端数据
        while (1) {
            int len = recv(client_sock, rx_buffer, sizeof(rx_buffer), 0);
            if (len < 0) {
                ESP_LOGE(TAG, "Error occurred during receiving");
                break;
            } else if (len == 0) {
                ESP_LOGI(TAG, "Connection closed");
                break;
            } else {
                // 直接转发接收到的数据，不添加结束符
                uart_write_bytes(UART_NUM_1, rx_buffer, len);
                
                // 打印客户端发送的信息
                ESP_LOGI(TAG, "Received data from client: %.*s", len, rx_buffer);
            }
        }

        // 关闭客户端连接
        shutdown(client_sock, 0);
        close(client_sock);
    }

    // 关闭监听socket
    close(listen_sock);
    vTaskDelete(NULL);
}
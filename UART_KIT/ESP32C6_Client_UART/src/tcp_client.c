#include "tcp_client.h"

#define SERVER_IP        "192.168.4.1"
#define SERVER_PORT      3333
#define SEND_INTERVAL_MS 2
static const char *TAG = "TCP_CLIENT";

static bool tcp_communication_test_loop(int sock);
// 处理TCP数据函数
void process_tcp_data(int sock) {
    // 接收数据
    char rx_buffer[128];
    int len = recv(sock, rx_buffer, sizeof(rx_buffer) - 1, 0);
    if (len > 0) {
        
    }
}

void tcp_client_task(void *pvParameters) {
    TaskHandle_t my_task_handle = xTaskGetCurrentTaskHandle();
    wifi_init_sta(my_task_handle);
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    int sock = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);
    if (sock < 0) {
        ESP_LOGE(TAG, "Failed to create TCP socket");
        goto exit;
    }

    struct sockaddr_in server_addr = {
        .sin_family = AF_INET,
        .sin_port = htons(SERVER_PORT),
        .sin_addr.s_addr = inet_addr(SERVER_IP)
    };

    if (connect(sock, (struct sockaddr *)&server_addr, sizeof(server_addr))) {
        ESP_LOGE(TAG, "TCP connect failed");
        close(sock);
        goto exit;
    }

    struct timeval tv = { .tv_sec = 0, .tv_usec = 50000 };
    setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    while (1) {

        //demo
        if(! tcp_communication_test_loop(sock)){
            break;
        }
        //use yout function:
        //process_tcp_data(sock);
    }

    close(sock);
exit:
    vTaskDelete(NULL);
}

static bool tcp_communication_test_loop(int sock) {
    const char *message = "2,Hello from TCP Client1!";
     // 接收数据
    char rx_buffer[128];
    // 发送数据s
    if (send(sock, message, strlen(message), 0) < 0) {
        ESP_LOGE(TAG, "TCP send failed");
        return false; // 返回false表示需要退出循环
    }
    int len = recv(sock, rx_buffer, sizeof(rx_buffer) - 1, 0);
    if (len > 0) {
        rx_buffer[len] = '\0';
        ESP_LOGI(TAG, "TCP received: %s", rx_buffer);
    } else if (len == 0) {
        ESP_LOGE(TAG, "TCP connection closed");
        return false; // 返回false表示需要退出循环
    }
    ESP_LOGI(TAG, "TCP sent: %s", message);
    vTaskDelay(pdMS_TO_TICKS(10));
    return true; // 返回true表示继续循环
}
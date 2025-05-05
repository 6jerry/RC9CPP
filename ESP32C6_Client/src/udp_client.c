#include "udp_client.h"

#define SERVER_IP        "192.168.4.1"
#define SERVER_PORT      3333
#define SEND_INTERVAL_MS 16 // 发送间隔（毫秒）
static const char *TAG = "UDP_CLIENT";
int64_t udp_get_data = 0;
bool udp_communication_test_loop(int sock, struct sockaddr_in *server_addr);
static long long cnt = 0;
static void wifi_event_handler(void* arg, esp_event_base_t event_base, 
                             int32_t event_id, void* event_data) {
    if (event_id == WIFI_EVENT_STA_DISCONNECTED) {
        ESP_LOGE(TAG, "WiFi disconnected! Restarting...");
        vTaskDelay(pdMS_TO_TICKS(100));
        esp_restart();
    }
}

// 新增绑定本地端口函数
static bool bind_local_port(int sock) {
    struct sockaddr_in local_addr = {
        .sin_family = AF_INET,
        .sin_port = htons(SERVER_PORT),
        .sin_addr.s_addr = INADDR_ANY
    };
    
    // int opt = 1;
    // setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
    
    int opt = 1;
    setsockopt(sock, SOL_SOCKET, SO_SNDTIMEO, &(struct timeval){.tv_usec = 2000}, sizeof(struct timeval));
    setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    if (bind(sock, (struct sockaddr *)&local_addr, sizeof(local_addr)) < 0) {
        ESP_LOGE(TAG, "Bind failed: errno %d", errno);
        return false;
    }
    ESP_LOGI(TAG, "Bound to port %d", SERVER_PORT);
    return true;
}

// 修改后的数据处理函数
// void process_udp_data(int sock) {
//     char rx_buffer[128];
//     struct sockaddr_in from_addr;
//     socklen_t from_len = sizeof(from_addr);

//     struct timeval tv = { .tv_sec = 0, .tv_usec = 20000 };
//     setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
    
//     int len = recvfrom(sock, rx_buffer, sizeof(rx_buffer)-1, 0,
//                      (struct sockaddr *)&from_addr, &from_len);
    
//     if (len > 0) {
//         if(rx_buffer[0] == '2' && rx_buffer[1] == ',' ){ // 接收到的是数据帧
//             rx_buffer[len] = '\0';
//             ESP_LOGI(TAG, "Received %s",rx_buffer);
//         }
//     }
// }

void process_udp_data(int sock) {
    static int64_t last_receive_time = 0; // 静态变量记录上次接收时间
    int64_t current_time = esp_timer_get_time();

    char rx_buffer[128];
    struct sockaddr_in from_addr;
    socklen_t from_len = sizeof(from_addr);

    struct timeval tv = { .tv_sec = 0, .tv_usec = 1000 }; // 1ms超时
    setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
    
    int len = recvfrom(sock, rx_buffer, sizeof(rx_buffer)-1, 0,
                     (struct sockaddr *)&from_addr, &from_len);
    
    if (len > 0) {
        rx_buffer[len] = '\0';
        if(rx_buffer[0] == '2' && rx_buffer[1] == ',' ) {
            // 计算接收间隔
            if (last_receive_time != 0) {
                int64_t interval_us = current_time - last_receive_time;
                ESP_LOGI(TAG, "接收间隔: %lld us (%.2f ms)", 
                        interval_us, (float)interval_us / 1000.0f);
            } else {
                ESP_LOGI(TAG, "首次接收到数据");
            }
            last_receive_time = current_time; // 更新接收时间戳

            // ESP_LOGI(TAG, "Received: %s", rx_buffer);
        }
    }
}


bool udp_sent = false;
void udp_client_task(void *pvParameters) {
    TaskHandle_t my_task_handle = xTaskGetCurrentTaskHandle();
    wifi_init_sta(my_task_handle);
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    if (sock < 0) {
        ESP_LOGE(TAG, "Failed to create UDP socket");
        goto exit;
    }
    // 绑定本地端口
    if (!bind_local_port(sock)) {
        close(sock);
        goto exit;
    }

    struct sockaddr_in server_addr = {
        .sin_family = AF_INET,
        .sin_port = htons(SERVER_PORT),
        .sin_addr.s_addr = inet_addr(SERVER_IP)
    };
    // 注册WiFi事件处理
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, 
        &wifi_event_handler, NULL));

    init_udp_client(sock, &server_addr);


    while (1) {

        if(fail_flag){
            sent_fail_count = 0; // 重置计数器
            close(sock); // 关闭套接字
            vTaskDelay(pdMS_TO_TICKS(1000)); // 等待1秒
            esp_restart(); // 重启ESP32
        }

        //use yout function:
        // if(esp_timer_get_time() - udp_get_data > 500000){
        //     process_udp_data(sock);
        //     udp_get_data = esp_timer_get_time();
        // }
        process_udp_data(sock);

        if(udp_sent){
            udp_sent = false;
            int64_t end = esp_timer_get_time();
            ESP_LOGI(TAG, "Time: %lld us", end - start);
            start = esp_timer_get_time();
        }

        vTaskDelay(pdMS_TO_TICKS(1));
    }

    close(sock);
exit:
    vTaskDelete(NULL);
}

bool udp_communication_test_loop(int sock, struct sockaddr_in *server_addr) {

    static int64_t last_send_time = 0; // 静态变量记录上次发送时间
    int64_t current_time = esp_timer_get_time();
    // 强制按固定频率发送
    if (current_time - last_send_time >= 16000) { // 16ms
        if(update_xbox){
            static uint8_t data[38] = "2,"; // 预构造包头
            memcpy(data+2, parser.current_frame.data, parser.current_frame.data_length); // 仅需拷贝数据部分
            // char data[38];
            // memcpy(data,"2,", 2);
            // memcpy(data + 2, parser.current_frame.data, parser.current_frame.data_length);

            int sent_bytes = sendto(sock, data, 38, 0, (struct sockaddr *)server_addr, sizeof(*server_addr));
            if (sent_bytes < 0) {
                ESP_LOGE(TAG, "Send failed, errno=%d", errno);
            } else {
                ESP_LOGI(TAG, "Sent %d bytes", sent_bytes);
            }

            int retry_count = 0;
            while (sent_bytes < 0) {
                ESP_LOGE(TAG, "UDP send failed, retry %d", retry_count);
                ESP_LOGE(TAG, "Send failed, errno=%d", errno);
                retry_count++;
                sent_bytes = sendto(sock, data, 38, 0, (struct sockaddr *)server_addr, sizeof(*server_addr));
                if (retry_count >= 3){
                    retry_count = 2;
                    break;
                } // 最多重试3次
                vTaskDelay(pdMS_TO_TICKS(10));
            }

            // 记录发送时间间隔
            // current_time = esp_timer_get_time();
            // if (last_send_time != 0) {
            //     int64_t interval_us = current_time - last_send_time;
            //     ESP_LOGI(TAG, "UDP发送间隔: %lld us (%.2f ms)", 
            //             interval_us, (float)interval_us / 1000.0f);
            // }
            // last_send_time = current_time; // 更新发送时间戳

            update_xbox = false;
            last_send_time = current_time;
            return true;
        }
    }
    return true; // 保持循环
}


/**
 * @brief 将HEX数组转换为字符串（带分隔符）
 * @param data      HEX数组指针
 * @param length    数组长度
 * @param delimiter 分隔符（如空格、逗号，若不需要填'\0'）
 * @return          生成的字符串指针（需手动释放内存）
 */
char* hex_to_string(const uint8_t* data, size_t length, char delimiter) {
    // 计算所需缓冲区大小：每个字节2字符 + 分隔符 + '\0'
    size_t buffer_size = (length * 2) + (length ? (length - 1) : 0) + 3;
    char* buffer = (char*)malloc(buffer_size);
    if (!buffer) return NULL;

    char* ptr = buffer;
    for (size_t i = 0; i < length; i++) {
        // 格式化当前字节为两位HEX（大写）
        sprintf(ptr, "%02X", data[i]);
        ptr += 2;

        // 添加分隔符（非最后一个元素）
        if (delimiter != '\0' && i < length - 1) {
            *ptr++ = delimiter;
        }
    }
    *ptr = '\n'; // 字符串结尾
    ptr++;
    *ptr = '\r';
    ptr++;
    *ptr = '\0'; // 字符串结尾

    return buffer;
}

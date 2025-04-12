#include "udp_server.h"
#include <stdio.h>
#include <string.h>
#include "esp_log.h"
#include "lwip/err.h"
#include "lwip/sys.h"
#include "lwip/sockets.h"
#include "device_info.h"
#include "driver/uart.h"

#define PORT 3333
#define NUM_DEVICES 4

static const char *UDP_TAG = "UDP_SERVER";  // 修改为UDP_TAG

//哪里使用哪里定义
// DeviceInfo devices[NUM_DEVICES];

union check_code
    {
        uint16_t crc_code;
        uint8_t crc_buff[2];
    } check_code;

// 计算 CRC16 校验码
uint16_t CRC16_Table(uint8_t *p, uint8_t counter)
{
    uint16_t crc = 0;
    for (uint8_t i = 0; i < counter; i++)
    {
        crc = CRC16Table[((crc >> 8) ^ p[i]) & 0xFF] ^ (crc << 8);
    }
    return crc;
}

// 发送数据帧的函数
void send_serial_frame_mat(uint8_t frame_id, uint8_t data_length, uint8_t *data)
{
    uint8_t buff_msg[data_length + 8];
    buff_msg[0] = FRAME_HEAD_0_MAT;
    buff_msg[1] = FRAME_HEAD_1_MAT;
    buff_msg[2] = frame_id;
    buff_msg[3] = data_length;

    for (int q = 0; q < data_length; q++)
    {
        buff_msg[4 + q] = data[q]; // 直接装填
    }

    check_code.crc_code = CRC16_Table(buff_msg, data_length * 4);
    buff_msg[4 + data_length] = check_code.crc_buff[0];
    buff_msg[5 + data_length] = check_code.crc_buff[1];
    buff_msg[6 + data_length] = FRAME_END_0_MAT;
    buff_msg[7 + data_length] = FRAME_END_1_MAT;

    // 使用UART发送数据
    uart_write_bytes(UART_NUM_1, (const char*)(buff_msg),  data_length + 8);
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
    int opt = 1;
    setsockopt(sock, SOL_SOCKET, SO_BROADCAST, &opt, sizeof(opt)); // 确保广播权限
    // 设置发送缓冲区大小（单位：字节）
    int send_buf_size = 8760; // 推荐值
    setsockopt(sock, SOL_SOCKET, SO_SNDBUF, &send_buf_size, sizeof(send_buf_size));
    // 设置接收缓冲区大小
    int recv_buf_size = 8760;
    setsockopt(sock, SOL_SOCKET, SO_RCVBUF, &recv_buf_size, sizeof(recv_buf_size));
    while (1) {
        struct sockaddr_in client_addr;
        socklen_t client_addr_len = sizeof(client_addr);
        char buffer[512];
        
        int len = recvfrom(sock, buffer, sizeof(buffer) - 1, 0, 
                           (struct sockaddr *)&client_addr, &client_addr_len);
        if (len > 0) {
            buffer[len] = '\0';
            //打印日志
            ESP_LOGI(UDP_TAG, "Received: %s", buffer);

            /****************调试信息****************/
            // send_serial_frame_mat(1 ,28, (uint8_t*)(buffer+2));
            // uart_write_bytes(UART_NUM_1, (const char*)(buffer+2),  28);

            // uart_write_bytes(UART_NUM_0, (const char*)(buffer+2),  28);
            // uart_write_bytes(UART_NUM_0, (const char*)"\r\n\0",  3);

            /***************** 效果差，不使用 ******************/
            // 获取客户端MAC地址
            // uint8_t mac[6];
            // 更新设备信息
            // update_device_info(mac, &client_addr);
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
            /*************************************************/

            // 广播消息到所有设备
            struct sockaddr_in broadcast_addr;
            broadcast_addr.sin_family = AF_INET;
            broadcast_addr.sin_port = htons(PORT);
            broadcast_addr.sin_addr.s_addr = htonl(INADDR_BROADCAST);
            
            int broadcast_len = sendto(sock, buffer, len, 0,
                                     (struct sockaddr *)&broadcast_addr, sizeof(broadcast_addr));
            usleep(100); // 100微秒延迟，避免 flooding
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

void udp_server_start(void) {
    init_devices();  // 初始化设备信息
    xTaskCreate(udp_server_task, "udp_server", 8192, NULL, 15, NULL);
}
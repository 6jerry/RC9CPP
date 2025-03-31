#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "nvs_flash.h"
#include "driver/uart.h"
#include "udp_server.h"
#include "tcp_server.h"
#include "lwip/ip4_addr.h"
#include "device_info.h"


#define PORT 3333
#define MAX_CLIENTS 5

#define NUM_DEVICES 4

#define MACSTR "%02x:%02x:%02x:%02x:%02x:%02x"
#define MAC2STR(a) (a)[0], (a)[1], (a)[2], (a)[3], (a)[4], (a)[5]

static const char *MAIN_TAG = "MAIN";  // 修改TAG为MAIN_TAG


// // 在wifi_event_handler中添加MAC地址获取
// static void wifi_event_handler(void* arg, esp_event_base_t event_base,
//                              int32_t event_id, void* event_data) {
//     if (event_id == WIFI_EVENT_AP_STACONNECTED) {
//         wifi_event_ap_staconnected_t* event = 
//             (wifi_event_ap_staconnected_t*) event_data;
//         ESP_LOGI(MAIN_TAG, "station " MACSTR " join, AID=%d",
//                 MAC2STR(event->mac), event->aid);
//     }
// }

// static void dhcp_event_handler(void* arg, esp_event_base_t event_base,
//                                int32_t event_id, void* event_data) {
//     if (event_id == IP_EVENT_AP_STAIPASSIGNED) {
//         ip_event_ap_staipassigned_t* event = 
//             (ip_event_ap_staipassigned_t*) event_data;

//         // 获取已连接设备列表
//         wifi_sta_list_t sta_list;
//         if (esp_wifi_ap_get_sta_list(&sta_list) != ESP_OK) {
//             ESP_LOGE(MAIN_TAG, "Failed to get STA list");
//             return;
//         }

//         // 遍历已连接设备列表，查找触发 DHCP 事件的设备
//         for (int i = 0; i < sta_list.num; i++) {
//             // 创建 sockaddr_in 结构体
//             struct sockaddr_in client_addr;
//             client_addr.sin_addr.s_addr = event->ip.addr; // 使用客户端的 IP 地址
//             client_addr.sin_family = AF_INET;

//             // 获取当前设备的 MAC 地址
//             uint8_t* mac = sta_list.sta[i].mac;

//             // 打印设备信息（可选，用于调试）
//             ESP_LOGI(MAIN_TAG, "Connected device MAC: %02x:%02x:%02x:%02x:%02x:%02x",
//                      mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

//             // 更新设备信息
//             update_device_info(mac, &client_addr);
//         }
//     } else if (event_id == WIFI_EVENT_STA_DISCONNECTED) {
//         // 处理设备断开事件
//         wifi_event_sta_disconnected_t* event = 
//             (wifi_event_sta_disconnected_t*) event_data;

//         // 获取断开设备的 MAC 地址
//         uint8_t* mac = event->bssid;

//         // 打印断开设备的 MAC 地址（可选，用于调试）
//         ESP_LOGI(MAIN_TAG, "Disconnected device MAC: %02x:%02x:%02x:%02x:%02x:%02x",
//                  mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

//         // 更新设备状态为非活跃
//         for (int i = 0; i < NUM_DEVICES; i++) {
//             if (memcmp(devices[i].mac, mac, 6) == 0) {
//                 devices[i].active = false;
//                 ESP_LOGI(MAIN_TAG, "Device marked as inactive: ID=%d", devices[i].id);
//                 break;
//             }
//         }
//     }
// }
static void wifi_init_softap(void) {
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    // ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL));
    // ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_AP_STAIPASSIGNED, &dhcp_event_handler, NULL));

    wifi_config_t wifi_config = {
        .ap = {
            .ssid = "ESP32C6_AP",
            .password = "123456789",
            .max_connection = MAX_CLIENTS,
            .authmode = WIFI_AUTH_WPA2_PSK
        },
    };

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_protocol(WIFI_IF_AP, WIFI_PROTOCOL_11B|WIFI_PROTOCOL_11G|WIFI_PROTOCOL_11N|WIFI_PROTOCOL_11AX));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(MAIN_TAG, "wifi_init_softap finished. SSID:%s password:%s",
             wifi_config.ap.ssid, wifi_config.ap.password);
}

void app_main(void) {
    // 设置日志串口波特率为921600
    uart_set_baudrate(UART_NUM_0, 921600);
    // 初始化NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // 初始化网络接口
    ESP_ERROR_CHECK(esp_netif_init());

    // 创建事件循环
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    // 创建默认WiFi AP网络接口
    esp_netif_create_default_wifi_ap();
    // 初始化WiFi AP
    wifi_init_softap();
    init_devices();
    //  启动服务器
    udp_server_start();
    // tcp_server_start();
}
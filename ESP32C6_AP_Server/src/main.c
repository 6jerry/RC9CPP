#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h" 
#include "nvs_flash.h"
#include "tcp_server.h"
#include "udp_server.h"
#include "lwip/ip4_addr.h"  // Add this include for IP4_ADDR macro

#define PORT 3333
#define MAX_CLIENTS 5

static const char *TAG = "TCP_SERVER";

static void wifi_init_softap(void)
{
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
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
    

    ESP_LOGI(TAG, "wifi_init_softap finished. SSID:%s password:%s",
             wifi_config.ap.ssid, wifi_config.ap.password);


}

void app_main(void)
{
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

    // 启动TCP服务器
    tcp_server_start();
    // udp_server_start();
}



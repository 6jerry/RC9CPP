#include <stdio.h>
#include <string.h>
#include "esp_wifi.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "tcp_server.h"
#include "driver/uart.h"
#include "driver/gpio.h"

#define WIFI_SSID "ESP32_AP" // WiFi AP的SSID
#define WIFI_PASS "12345678"  // WiFi AP的密码

// 初始化WiFi AP模式
static void wifi_init_softap(void)
{
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    // 配置WiFi AP参数
    wifi_config_t wifi_config = {
        .ap = {
            .ssid = WIFI_SSID,
            .ssid_len = strlen(WIFI_SSID),
            .password = WIFI_PASS,
            .max_connection = 4, // 最大连接数
            .authmode = WIFI_AUTH_WPA_WPA2_PSK // 认证模式
        },
    };

    // 设置WiFi模式为AP并启动
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_AP, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
}

// 主程序入口
void app_main(void)
{
    // 初始化NVS（非易失性存储）
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // 配置并初始化UART1
    uart_config_t uart_config = {
        .baud_rate = 115200, // 波特率
        .data_bits = UART_DATA_8_BITS, // 数据位
        .parity = UART_PARITY_DISABLE, // 无校验
        .stop_bits = UART_STOP_BITS_1, // 停止位
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE, // 无硬件流控
    };
    ESP_ERROR_CHECK(uart_param_config(UART_NUM_1, &uart_config));
    // 设置UART1引脚：TX=GPIO17, RX=GPIO18
    ESP_ERROR_CHECK(uart_set_pin(UART_NUM_1, GPIO_NUM_17, GPIO_NUM_18, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM_1, 1024 * 2, 0, 0, NULL, 0));

    // 初始化WiFi网络接口
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_ap();

    // 启动WiFi AP
    wifi_init_softap();

    // 创建TCP服务器任务
    xTaskCreate(tcp_server_task, "tcp_server", 4096, NULL, 5, NULL);
}
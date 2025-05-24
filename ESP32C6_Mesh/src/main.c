#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "nvs_flash.h"
#include "esp_mesh.h"
#include "udp_server.h"

#define PORT 3333
#define MAX_CLIENTS 5
#define NUM_DEVICES 4
#define MACSTR "%02x:%02x:%02x:%02x:%02x:%02x"
#define MAC2STR(a) (a)[0], (a)[1], (a)[2], (a)[3], (a)[4], (a)[5]
#define BUF_SIZE (1024)

static const char *MAIN_TAG = "MAIN";

// 定义 Mesh 网络的配置参数

uint8_t MESH_ID[6] = {0x12, 0x34, 0x56, 0x78, 0x90, 0xab};
#define CONFIG_MESH_CHANNEL 6
#define CONFIG_MESH_ROUTER_SSID "ESP32C6_AP"
#define CONFIG_MESH_ROUTER_PASSWD "123456789"
#define CONFIG_MESH_AP_CONNECTIONS 5
#define CONFIG_MESH_AP_PASSWD "123456789"

// IP 事件处理程序
static void ip_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data) {
    if (event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        ESP_LOGI(MAIN_TAG, "Got IP Address: " IPSTR, IP2STR(&event->ip_info.ip));
    }
}

// Mesh 事件处理程序
static void mesh_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data) {
    switch (event_id) {
        case MESH_EVENT_STARTED:
            ESP_LOGI(MAIN_TAG, "Mesh started");
            break;
        case MESH_EVENT_STOPPED:
            ESP_LOGI(MAIN_TAG, "Mesh stopped");
            break;
        case MESH_EVENT_CHILD_CONNECTED:
            ESP_LOGI(MAIN_TAG, "Child connected");
            break;
        case MESH_EVENT_CHILD_DISCONNECTED:
            ESP_LOGI(MAIN_TAG, "Child disconnected");
            break;
        case MESH_EVENT_PARENT_CONNECTED:
            ESP_LOGI(MAIN_TAG, "Parent connected");
            break;
        case MESH_EVENT_PARENT_DISCONNECTED:
            ESP_LOGI(MAIN_TAG, "Parent disconnected");
            break;
        case MESH_EVENT_ROUTING_TABLE_ADD:
            ESP_LOGI(MAIN_TAG, "Routing table added");
            break;
        case MESH_EVENT_ROUTING_TABLE_REMOVE:
            ESP_LOGI(MAIN_TAG, "Routing table removed");
            break;
        case MESH_EVENT_NO_PARENT_FOUND:
            ESP_LOGI(MAIN_TAG, "No parent found");
            break;
        case MESH_EVENT_LAYER_CHANGE:
            ESP_LOGI(MAIN_TAG, "Layer changed");
            break;
        case MESH_EVENT_TODS_STATE:
            ESP_LOGI(MAIN_TAG, "ToDS state changed");
            break;
        default:
            ESP_LOGI(MAIN_TAG, "Mesh event: %ld", event_id);
            break;
    }
}

// 初始化 Mesh
static void mesh_init(void) {
    // 初始化 Mesh
    ESP_ERROR_CHECK(esp_mesh_init());

    // 注册 Mesh 事件处理程序
    ESP_ERROR_CHECK(esp_event_handler_register(MESH_EVENT, ESP_EVENT_ANY_ID, &mesh_event_handler, NULL));

    // 配置 Mesh 网络
    mesh_cfg_t cfg = MESH_INIT_CONFIG_DEFAULT();
    memcpy((uint8_t *) &cfg.mesh_id, MESH_ID, 6);  // 设置 Mesh ID
    cfg.channel = CONFIG_MESH_CHANNEL;  // 设置信道
    cfg.router.ssid_len = strlen(CONFIG_MESH_ROUTER_SSID);
    memcpy((uint8_t *) &cfg.router.ssid, CONFIG_MESH_ROUTER_SSID, cfg.router.ssid_len);  // 设置路由器 SSID
    memcpy((uint8_t *) &cfg.router.password, CONFIG_MESH_ROUTER_PASSWD, strlen(CONFIG_MESH_ROUTER_PASSWD));  // 设置路由器密码
    cfg.mesh_ap.max_connection = CONFIG_MESH_AP_CONNECTIONS;  // 设置 Mesh AP 最大连接数
    memcpy((uint8_t *) &cfg.mesh_ap.password, CONFIG_MESH_AP_PASSWD, strlen(CONFIG_MESH_AP_PASSWD));  // 设置 Mesh AP 密码

    ESP_ERROR_CHECK(esp_mesh_set_config(&cfg));  // 设置 Mesh 配置

    // 启动 Mesh
    ESP_ERROR_CHECK(esp_mesh_start());

    ESP_LOGI(MAIN_TAG, "Mesh initialized and started");
}

void app_main(void) {
    // 初始化 NVS
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

    // 初始化 Wi-Fi
    wifi_init_config_t config = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&config));

    // 注册 IP 事件处理程序
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &ip_event_handler, NULL));

    // 设置 Wi-Fi 存储模式
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_FLASH));

    // 启动 Wi-Fi
    ESP_ERROR_CHECK(esp_wifi_start());

    // 初始化 Mesh
    mesh_init();

    esp_wifi_set_ps(WIFI_PS_NONE);  // 禁用节能模式
    esp_wifi_set_max_tx_power(84);  // 84 * 0.25 = 21dBm（最大合法功率）
    udp_server_start();
}
#include "device_info.h"

static const char *DEVICE_TAG = "DEVICE";  // 修改为TCP_TAG
void init_devices() {
    // 示例MAC地址和ID，您可以根据需要修改
    uint8_t mac1[6] = {0x40, 0x4c, 0xca, 0x5e, 0x5d, 0x74};
    uint8_t mac2[6] = {0x40, 0x4c, 0xca, 0x5e, 0x8c, 0x40};
    uint8_t mac3[6] = {0x40, 0x4c, 0xca, 0x55, 0x36, 0x91};
    uint8_t mac4[6] = {0x00, 0x11, 0x22, 0x33, 0x44, 0x88};

    memcpy(devices[0].mac, mac1, 6);
    devices[0].id = 1;
    devices[0].active = false;

    memcpy(devices[1].mac, mac2, 6);
    devices[1].id = 2;
    devices[1].active = false;

    memcpy(devices[2].mac, mac3, 6);
    devices[2].id = 3;
    devices[2].active = false;

    memcpy(devices[3].mac, mac4, 6);
    devices[3].id = 4;
    devices[3].active = false;
}

// 更新设备信息
bool is_ip_already_assigned(struct sockaddr_in *addr) {
    for (int i = 0; i < NUM_DEVICES; i++) {
        if (devices[i].active && devices[i].addr.sin_addr.s_addr == addr->sin_addr.s_addr) {
            return true;
        }
    }
    return false;
}

void update_device_info(const uint8_t *mac, struct sockaddr_in *addr) {
    for (int i = 0; i < NUM_DEVICES; i++) {
        if (memcmp(devices[i].mac, mac, 6) == 0) {
            // 如果设备已经是活跃状态，且已经分配了 IP，则不更新
            if (devices[i].active && devices[i].addr.sin_addr.s_addr != 0) {
                ESP_LOGI(DEVICE_TAG, "Device is already active with IP. No update needed: ID=%d, IP=%s",
                         devices[i].id, inet_ntoa(devices[i].addr.sin_addr));
            } else {
                // 检查 IP 地址是否已经被分配给其他设备
                if (is_ip_already_assigned(addr)) {
                    ESP_LOGE(DEVICE_TAG, "IP address %s is already assigned to another device. Update aborted.",
                             inet_ntoa(addr->sin_addr));
                } else {
                    // 更新 IP 地址并标记为活跃
                    devices[i].addr = *addr;
                    devices[i].active = true;
                    ESP_LOGI(DEVICE_TAG, "Device updated: ID=%d, IP=%s, Port=%d",
                             devices[i].id, inet_ntoa(addr->sin_addr), ntohs(addr->sin_port));
                }
            }
            return;
        }
    }
    ESP_LOGE(DEVICE_TAG, "Unknown device with MAC: %02x:%02x:%02x:%02x:%02x:%02x",
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
}

/* 设备 B（ESP32 S3）代码 */

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_event.h"
#include "esp_wifi.h"
#include "esp_now.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_system.h"

// 日志标签
static const char *TAG = "Device B";

// 将此处替换为设备 A（ESP32 WROOM-32）的实际 MAC 地址
uint8_t peer_mac[ESP_NOW_ETH_ALEN] = { 0x11, 0x22, 0x33, 0x44, 0x55, 0x66 };  // 占位符

// ESPNOW 接收回调函数
void espnow_recv_cb(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len)
{
    if (data == NULL || len <= 0) {
        ESP_LOGE(TAG, "接收回调错误：数据为空或长度 <= 0");
        return;
    }

    // 将接收到的数据转换为字符串
    char message[250];
    if (len >= sizeof(message)) {
        len = sizeof(message) - 1;
    }
    memcpy(message, data, len);
    message[len] = '\0';  // 确保以 null 终止

    ESP_LOGI(TAG, "收到来自 " MACSTR " 的消息: %s", MAC2STR(recv_info->src_addr), message);

    // 处理消息
    if (strcmp(message, "Halfapear") == 0) {
        ESP_LOGI(TAG, "收到定时消息。");
        // 添加处理定时消息的代码
    } else if (strcmp(message, "Halfapear 获得高电平") == 0) {
        ESP_LOGI(TAG, "收到高电平触发消息。");
        // 添加处理高电平触发的代码
    } else {
        ESP_LOGW(TAG, "收到未知消息。");
    }
}

// 初始化 Wi-Fi 为站点模式
static void wifi_init(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_t *sta_netif = esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    // 设置 Wi-Fi 模式为站点模式
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());
}

// 初始化 ESPNOW
static void espnow_init(void)
{
    // 初始化 ESPNOW
    ESP_ERROR_CHECK(esp_now_init());
    // 注册接收回调函数
    ESP_ERROR_CHECK(esp_now_register_recv_cb(espnow_recv_cb));

    // 如果需要，设置主密钥（PMK）
    // ESP_ERROR_CHECK(esp_now_set_pmk((uint8_t *)"pmk1234567890123"));

    // 添加对端信息
    esp_now_peer_info_t peerInfo = {};
    memcpy(peerInfo.peer_addr, peer_mac, ESP_NOW_ETH_ALEN);
    peerInfo.channel = 0;  // 0 表示当前频道，或设置特定频道
    peerInfo.ifidx = ESP_IF_WIFI_STA;
    peerInfo.encrypt = false;  // 如果需要加密，设置为 true

    // 添加对端
    ESP_ERROR_CHECK(esp_now_add_peer(&peerInfo));
}

void app_main(void)
{
    // 初始化 NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        // NVS 分区已损坏，需要擦除
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // 初始化 Wi-Fi
    wifi_init();

    // 打印自身的 MAC 地址
    uint8_t mac[6];
    ESP_ERROR_CHECK(esp_read_mac(mac, ESP_MAC_WIFI_STA));
    ESP_LOGI(TAG, "设备 B MAC 地址: " MACSTR, MAC2STR(mac));

    // 初始化 ESPNOW
    espnow_init();
}

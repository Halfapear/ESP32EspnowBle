#include <string.h>
#include <stdio.h>
#include "esp_now.h"
#include "esp_wifi.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "driver/gpio.h"
#include "esp_netif.h"
#include "esp_event.h"

#define LED_GPIO GPIO_NUM_48  // LED 连接的 GPIO 引脚

static const char *TAG = "ESP-NOW Receiver";

// 发送器的 MAC 地址（已更新为实际地址）
uint8_t peer_addr[6] = {0x2C, 0xBC, 0xBB, 0x4C, 0x66, 0x1C};

// 接收回调函数
void on_receive(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len) {
    if (len < 1) {
        ESP_LOGW(TAG, "Received empty message");
        return;
    }

    // 将接收到的数据转换为字符串
    char message[len + 1];
    memcpy(message, data, len);
    message[len] = '\0';

    ESP_LOGI(TAG, "Received message: %s", message);

    if (strcmp(message, "led on") == 0) {
        gpio_set_level(LED_GPIO, 1);
        ESP_LOGI(TAG, "LED turned ON");
    } else if (strcmp(message, "led off") == 0) {
        gpio_set_level(LED_GPIO, 0);
        ESP_LOGI(TAG, "LED turned OFF");
    } else {
        ESP_LOGW(TAG, "Unknown command");
    }
}

void init_esp_now() {
    // 初始化 ESP-NOW
    ESP_ERROR_CHECK(esp_now_init());

    // 注册接收回调函数
    ESP_ERROR_CHECK(esp_now_register_recv_cb(on_receive));

    // 添加对等设备
    esp_now_peer_info_t peer_info = {0};
    memcpy(peer_info.peer_addr, peer_addr, 6);
    peer_info.channel = 0;  // 使用默认频道
    peer_info.encrypt = false;

    ESP_ERROR_CHECK(esp_now_add_peer(&peer_info));

    ESP_LOGI(TAG, "ESP-NOW initialized and peer added");
}

void init_gpio() {
    // 配置 LED 引脚为输出
    // 如果需要调用 gpio_pad_select_gpio，可以使用 esp_rom_gpio_pad_select_gpio
    // esp_rom_gpio_pad_select_gpio(LED_GPIO);  // 可选，根据需要启用

    gpio_set_direction(LED_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_level(LED_GPIO, 0);  // 初始关闭 LED
}

void app_main(void) {
    // 初始化 NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // 初始化网络接口和事件循环
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    // 配置 Wi-Fi 为 STA 模式
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());

    // 初始化 GPIO
    init_gpio();

    // 初始化 ESP-NOW
    init_esp_now();

    ESP_LOGI(TAG, "Receiver is running...");
}

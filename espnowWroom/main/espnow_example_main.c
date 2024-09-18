/* 设备 A（ESP32 WROOM-32）代码 */

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "esp_event.h"
#include "esp_wifi.h"
#include "esp_now.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_system.h"

// 日志标签
static const char *TAG = "Device A";

// GPIO 配置
#define GPIO_INPUT_IO     4  // 输入引脚编号（根据需要更改）
#define GPIO_INPUT_PIN_SEL  (1ULL<<GPIO_INPUT_IO)
static xQueueHandle gpio_evt_queue = NULL;

// ESPNOW 参数
#define ESPNOW_MAX_DELAY 1000

// 将此处替换为设备 B（ESP32 S3）的实际 MAC 地址
uint8_t peer_mac[ESP_NOW_ETH_ALEN] = { 0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF };  // 占位符

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

// GPIO 中断服务程序
static void IRAM_ATTR gpio_isr_handler(void* arg)
{
    uint32_t gpio_num = (uint32_t) arg;
    // 将 GPIO 编号发送到队列
    xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
}

// 处理 GPIO 事件的任务
static void gpio_task(void* arg)
{
    uint32_t io_num;
    while (1) {
        // 等待 GPIO 事件
        if (xQueueReceive(gpio_evt_queue, &io_num, portMAX_DELAY)) {
            ESP_LOGI(TAG, "GPIO[%d] 触发，发送高电平消息", io_num);
            // 发送高电平消息
            const char *message = "Halfapear 获得高电平";
            esp_err_t result = esp_now_send(peer_mac, (const uint8_t *)message, strlen(message) + 1);
            if (result == ESP_OK) {
                ESP_LOGI(TAG, "高电平消息发送成功");
            } else {
                ESP_LOGE(TAG, "发送高电平消息错误: %s", esp_err_to_name(result));
            }
        }
    }
}

// 初始化 GPIO
static void gpio_init(void)
{
    gpio_config_t io_conf = {};
    // 配置 GPIO 引脚为输入，上拉，下降沿中断
    io_conf.intr_type = GPIO_INTR_POSEDGE;  // 上升沿中断
    io_conf.mode = GPIO_MODE_INPUT;         // 输入模式
    io_conf.pin_bit_mask = GPIO_INPUT_PIN_SEL;  // 选择引脚
    io_conf.pull_down_en = 0;               // 禁用下拉
    io_conf.pull_up_en = 1;                 // 启用上拉
    gpio_config(&io_conf);

    // 创建一个队列来处理 GPIO 事件
    gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));
    // 启动一个任务来处理 GPIO 事件
    xTaskCreate(gpio_task, "gpio_task", 2048, NULL, 10, NULL);

    // 安装 GPIO ISR 服务
    gpio_install_isr_service(0);
    // 为特定的 GPIO 引脚添加中断处理程序
    gpio_isr_handler_add(GPIO_INPUT_IO, gpio_isr_handler, (void*) GPIO_INPUT_IO);
}

// ESPNOW 发送回调函数
void espnow_send_cb(const uint8_t *mac_addr, esp_now_send_status_t status)
{
    if (mac_addr == NULL) {
        ESP_LOGE(TAG, "发送回调错误：MAC 地址为空");
        return;
    }

    ESP_LOGI(TAG, "最后一个包已发送到: " MACSTR ", 状态: %s", MAC2STR(mac_addr), status == ESP_NOW_SEND_SUCCESS ? "成功" : "失败");
}

// 定时发送任务
void periodic_send_task(void *pvParameter)
{
    while (1) {
        vTaskDelay(5000 / portTICK_PERIOD_MS);  // 延迟 5 秒

        // 发送定时消息
        const char *message = "Halfapear";
        esp_err_t result = esp_now_send(peer_mac, (const uint8_t *)message, strlen(message) + 1);
        if (result == ESP_OK) {
            ESP_LOGI(TAG, "定时消息发送成功");
        } else {
            ESP_LOGE(TAG, "发送定时消息错误: %s", esp_err_to_name(result));
        }
    }
}

// 初始化 ESPNOW
static void espnow_init(void)
{
    // 初始化 ESPNOW
    ESP_ERROR_CHECK(esp_now_init());
    // 注册发送回调函数
    ESP_ERROR_CHECK(esp_now_register_send_cb(espnow_send_cb));

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

    // 初始化 GPIO
    gpio_init();

    // 打印自身的 MAC 地址
    uint8_t mac[6];
    ESP_ERROR_CHECK(esp_read_mac(mac, ESP_MAC_WIFI_STA));
    ESP_LOGI(TAG, "设备 A MAC 地址: " MACSTR, MAC2STR(mac));

    // 初始化 ESPNOW
    espnow_init();

    // 启动定时发送任务
    xTaskCreate(periodic_send_task, "periodic_send_task", 2048, NULL, 4, NULL);
}

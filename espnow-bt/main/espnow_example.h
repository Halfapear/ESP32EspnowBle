/* espnow_example.h */

#ifndef ESPNOW_EXAMPLE_H
#define ESPNOW_EXAMPLE_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_now.h"
#include "esp_wifi.h"
#include "esp_bt.h"
#include "esp_gatt_common_api.h"

/* ESPNOW 可以在 station 和 softap 模式下工作，在 menuconfig 中配置。 */
#if CONFIG_ESPNOW_WIFI_MODE_STATION
#define ESPNOW_WIFI_MODE WIFI_MODE_STA
#define ESPNOW_WIFI_IF   ESP_IF_WIFI_STA
#else
#define ESPNOW_WIFI_MODE WIFI_MODE_AP
#define ESPNOW_WIFI_IF   ESP_IF_WIFI_AP
#endif

#define ESPNOW_QUEUE_SIZE           6

#define IS_BROADCAST_ADDR(addr) (memcmp(addr, s_example_broadcast_mac, ESP_NOW_ETH_ALEN) == 0)

/* ESPNOW 事件标识符 */
typedef enum {
    EXAMPLE_ESPNOW_SEND_CB,
    EXAMPLE_ESPNOW_RECV_CB,
} example_espnow_event_id_t;

/* ESPNOW 发送回调事件结构 */
typedef struct {
    uint8_t mac_addr[ESP_NOW_ETH_ALEN];
    esp_now_send_status_t status;
} example_espnow_event_send_cb_t;

/* ESPNOW 接收回调事件结构 */
typedef struct {
    uint8_t mac_addr[ESP_NOW_ETH_ALEN];
    uint8_t *data;
    int data_len;
} example_espnow_event_recv_cb_t;

/* ESPNOW 事件信息的联合体 */
typedef union {
    example_espnow_event_send_cb_t send_cb;
    example_espnow_event_recv_cb_t recv_cb;
} example_espnow_event_info_t;

/* ESPNOW 事件结构 */
typedef struct {
    example_espnow_event_id_t id;
    example_espnow_event_info_t info;
} example_espnow_event_t;

/* ESPNOW 数据类型 */
enum {
    EXAMPLE_ESPNOW_DATA_BROADCAST,
    EXAMPLE_ESPNOW_DATA_UNICAST,
    EXAMPLE_ESPNOW_DATA_MAX,
};

/* 自定义的 ESPNOW 数据结构 */
typedef struct {
    uint8_t type;                         // 广播或单播 ESPNOW 数据
    uint8_t state;                        // 指示是否已接收到广播 ESPNOW 数据
    uint16_t seq_num;                     // ESPNOW 数据的序列号
    uint16_t crc;                         // ESPNOW 数据的 CRC16 值
    uint32_t magic;                       // 用于确定哪个设备发送单播 ESPNOW 数据的魔术数字
    uint8_t payload[0];                   // ESPNOW 数据的实际有效载荷
} __attribute__((packed)) example_espnow_data_t;

/* 发送 ESPNOW 数据的参数 */
typedef struct {
    bool unicast;                         // 发送单播 ESPNOW 数据
    bool broadcast;                       // 发送广播 ESPNOW 数据
    uint8_t state;                        // 指示是否已接收到广播 ESPNOW 数据
    uint32_t magic;                       // 用于确定哪个设备发送单播 ESPNOW 数据的魔术数字
    uint16_t count;                       // 要发送的单播 ESPNOW 数据的总数
    uint16_t delay;                       // 发送两条 ESPNOW 数据之间的延迟，单位：ms
    int len;                              // 要发送的 ESPNOW 数据的长度，单位：字节
    uint8_t *buffer;                      // 指向 ESPNOW 数据的缓冲区
    uint8_t dest_mac[ESP_NOW_ETH_ALEN];   // 目标设备的 MAC 地址
} example_espnow_send_param_t;

/* BLE 相关定义 */

// 最大特征值长度
#define GATTS_DEMO_CHAR_VAL_LEN_MAX 0x40

// 准备写入缓冲区大小
#define PREPARE_BUF_MAX_SIZE 1024

// 配置文件数量
#define PROFILE_NUM 1
#define PROFILE_APP_ID 0

// 设备名称
#define DEVICE_NAME "ESP_NOW_BLE_DEMO"

// BLE 服务和特征 UUID
#define SERVICE_UUID   0x00FF
#define CHAR_UUID      0xFF01

// BLE 连接参数
#define MIN_CONN_INTERVAL 0x0006
#define MAX_CONN_INTERVAL 0x0010
#define SLAVE_LATENCY     0
#define CONN_SUP_TIMEOUT  400

/* GATT 配置文件实例结构体 */
struct gatts_profile_inst {
    esp_gatts_cb_t gatts_cb;
    uint16_t gatts_if;                    // GATT 接口句柄
    uint16_t app_id;                      // 应用程序 ID
    uint16_t conn_id;                     // 连接 ID
    uint16_t service_handle;              // 服务句柄
    esp_gatt_srvc_id_t service_id;        // 服务 ID
    uint16_t char_handle;                 // 特征句柄
    esp_bt_uuid_t char_uuid;              // 特征 UUID
    esp_gatt_perm_t perm;                 // 权限
    esp_gatt_char_prop_t property;        // 属性
    uint16_t descr_handle;                // 描述符句柄
    esp_bt_uuid_t descr_uuid;             // 描述符 UUID
};

/* 准备写入类型环境变量 */
typedef struct {
    uint8_t                 *prepare_buf;
    int                     prepare_len;
} prepare_type_env_t;

#endif /* ESPNOW_EXAMPLE_H */

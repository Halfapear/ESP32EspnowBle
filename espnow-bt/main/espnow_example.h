/* espnow_example.h */

#ifndef ESPNOW_EXAMPLE_H
#define ESPNOW_EXAMPLE_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_now.h"
#include "esp_wifi.h"
#include "esp_bt.h"
#include "esp_gatt_common_api.h"

/* ESPNOW can work in both station and softap mode. It is configured in menuconfig. */
#if CONFIG_ESPNOW_WIFI_MODE_STATION
#define ESPNOW_WIFI_MODE WIFI_MODE_STA
#define ESPNOW_WIFI_IF   ESP_IF_WIFI_STA
#else
#define ESPNOW_WIFI_MODE WIFI_MODE_AP
#define ESPNOW_WIFI_IF   ESP_IF_WIFI_AP
#endif

#define ESPNOW_QUEUE_SIZE           6

#define IS_BROADCAST_ADDR(addr) (memcmp(addr, s_example_broadcast_mac, ESP_NOW_ETH_ALEN) == 0)

/* ESPNOW event identifiers */
typedef enum {
    EXAMPLE_ESPNOW_SEND_CB,
    EXAMPLE_ESPNOW_RECV_CB,
} example_espnow_event_id_t;

/* ESPNOW send callback event structure */
typedef struct {
    uint8_t mac_addr[ESP_NOW_ETH_ALEN];
    esp_now_send_status_t status;
} example_espnow_event_send_cb_t;

/* ESPNOW receive callback event structure */
typedef struct {
    uint8_t mac_addr[ESP_NOW_ETH_ALEN];
    uint8_t *data;
    int data_len;
} example_espnow_event_recv_cb_t;

/* Union of ESPNOW event information */
typedef union {
    example_espnow_event_send_cb_t send_cb;
    example_espnow_event_recv_cb_t recv_cb;
} example_espnow_event_info_t;

/* ESPNOW event structure */
typedef struct {
    example_espnow_event_id_t id;
    example_espnow_event_info_t info;
} example_espnow_event_t;

/* ESPNOW data types */
enum {
    EXAMPLE_ESPNOW_DATA_BROADCAST,
    EXAMPLE_ESPNOW_DATA_UNICAST,
    EXAMPLE_ESPNOW_DATA_MAX,
};

/* User defined field of ESPNOW data in this example. */
typedef struct {
    uint8_t type;                         // Broadcast or unicast ESPNOW data.
    uint8_t state;                        // Indicate that if has received broadcast ESPNOW data or not.
    uint16_t seq_num;                     // Sequence number of ESPNOW data.
    uint16_t crc;                         // CRC16 value of ESPNOW data.
    uint32_t magic;                       // Magic number which is used to determine which device to send unicast ESPNOW data.
    uint8_t payload[0];                   // Real payload of ESPNOW data.
} __attribute__((packed)) example_espnow_data_t;

/* Parameters of sending ESPNOW data. */
typedef struct {
    bool unicast;                         // Send unicast ESPNOW data.
    bool broadcast;                       // Send broadcast ESPNOW data.
    uint8_t state;                        // Indicate that if has received broadcast ESPNOW data or not.
    uint32_t magic;                       // Magic number which is used to determine which device to send unicast ESPNOW data.
    uint16_t count;                       // Total count of unicast ESPNOW data to be sent.
    uint16_t delay;                       // Delay between sending two ESPNOW data, unit: ms.
    int len;                              // Length of ESPNOW data to be sent, unit: byte.
    uint8_t *buffer;                      // Buffer pointing to ESPNOW data.
    uint8_t dest_mac[ESP_NOW_ETH_ALEN];   // MAC address of destination device.
} example_espnow_send_param_t;

/* BLE related definitions */

// Maximum length of characteristic value
#define GATTS_DEMO_CHAR_VAL_LEN_MAX 0x40

// Prepare write buffer size
#define PREPARE_BUF_MAX_SIZE 1024

// Profile indices
#define PROFILE_NUM 1
#define PROFILE_APP_ID 0

// Device name
#define DEVICE_NAME "ESP_NOW_BLE_DEMO"

// BLE service and characteristic UUIDs
#define SERVICE_UUID   0x00FF
#define CHAR_UUID      0xFF01

// BLE connection parameters
#define MIN_CONN_INTERVAL 0x0006
#define MAX_CONN_INTERVAL 0x0010
#define SLAVE_LATENCY     0
#define CONN_SUP_TIMEOUT  400

/* Structure to store the GATT profile instance */
struct gatts_profile_inst {
    esp_gatts_cb_t gatts_cb;
    uint16_t gatts_if;                    // GATT interface handle
    uint16_t app_id;                      // Application ID
    uint16_t conn_id;                     // Connection ID
    uint16_t service_handle;              // Service handle
    esp_gatt_srvc_id_t service_id;        // Service ID
    uint16_t char_handle;                 // Characteristic handle
    esp_bt_uuid_t char_uuid;              // Characteristic UUID
    esp_gatt_perm_t perm;                 // Permissions
    esp_gatt_char_prop_t property;        // Properties
    uint16_t descr_handle;                // Descriptor handle
    esp_bt_uuid_t descr_uuid;             // Descriptor UUID
};

/* Prepare type environment variable */
typedef struct {
    uint8_t                 *prepare_buf;
    int                     prepare_len;
} prepare_type_env_t;

#endif /* ESPNOW_EXAMPLE_H */

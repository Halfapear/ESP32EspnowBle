#include "esp_system.h"
#include "Arduino.h"

void setup() {
  // 初始化串口通讯，波特率设置为115200
  Serial.begin(115200);

  // 定义一个数组来存储 MAC 地址
  uint8_t mac[6];
  
  // 获取 Wi-Fi MAC 地址：就是用个库函数 挺方便
  esp_efuse_mac_get_default(mac);
  
  // 打印 MAC 地址
  Serial.print("ESP32 MAC 地址: ");
  for (int i = 0; i < 6; i++) {
    if (mac[i] < 16) {
      Serial.print("0");
    }
    Serial.print(mac[i], HEX);
    if (i < 5) {
      Serial.print(":");
    }
  }
  Serial.println();
}

void loop() {
  // 空循环
}

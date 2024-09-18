## [gatt_server](https://github.com/Halfapear/ESP32bt-espnow/tree/main/gatt_server) 测试完成的ble；
功能包括：
- 在设备连接后每隔 5 秒发送一次数据 发送 嘟噜噜 连接成功×N N是计的数；
- 在我手机端输入任何东西时 esp32 都会输出字符串 “Halfapear收到信息啦”；

## [espnowPY](https://github.com/Halfapear/ESP32bt-espnow/tree/main/espnowPY) 测试完成的espnow&mac地址读取（micropython版）；
功能包括：
- 在wroom-32处按boot按钮 在s3处亮灯，同时两端输出日志led on/off

## [espnowS3](https://github.com/Halfapear/ESP32bt-espnow/tree/main/espnowS3) 测试完成的espnow（s3esp版）

## [EspnowBleS3](https://github.com/Halfapear/ESP32bt-espnow/tree/main/EspnowBleS3) 测试完成的s3的espnow&ble；
- 是上面的融合 s3端；
- 解决了一些内存问题

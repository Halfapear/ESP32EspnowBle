
#### 接收端的代码
import network
import espnow
from machine import Pin

sta = network.WLAN(network.STA_IF)
sta.active(True)
sta.disconnect()                # 断开WIFI连接

e = espnow.ESPNow()                  # 启动ESPNOW
e.active(True)

peer = b',\xbc\xbbLf\x1c'   # 发送器的mac地址
e.add_peer(peer)

led = Pin(48,Pin.OUT)

def espnow_rx():
    led.value(1)
    while True:
        host, msg = e.recv()
        if msg == b'led on':
            # 等待消息
            led.value(0)
            print(msg)

        elif msg == b'led off':
            led.value(1)
            print(msg)

if __name__ == "__main__":
    espnow_rx()


#发送端
import network
import espnow
import time
from machine import Pin

i = 0
sta = network.WLAN(network.STA_IF)    # 启用站点模式
sta.active(True)
sta.disconnect()        # 断开WIFI连接

e = espnow.ESPNow()     # 启动ESPNOW
e.active(True)
peer1 = b'\x80e\x99\xdf\xd2\\'  # 接收器的mac地址

e.add_peer(peer1) #如果有多个接收器就增加peer2...

print("Starting...") 
key_up = Pin(0,Pin.IN, Pin.PULL_UP)

def debounce(pin):
    state = pin.value()
    time.sleep(0.01)
    return state == pin.value()

def main():
    global i
    while True :
        if debounce(key_up) and key_up.value() == 0:  # 如果按键被按下
            if not key_up.value():               # 如果按键被按下
                e.send(peer1, "led on", True)
                while not key_up.value():            # 当按键没抬起之前一直死循环在这里
                    pass
            i = 0               
        elif not (debounce(key_up) and key_up.value() == 0): # 放开按键
            if i >0:
                pass
            else:
                e.send(peer1, "led off", True)
                print("'led off' ")  # 打印信息到电脑屏幕
            i+=1

if __name__ == "__main__":
    main()

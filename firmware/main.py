from machine import Pin, PWM, I2C

from ads1x15 import ADS1115

from utime import sleep

i2c = I2C(0, sda=Pin(21), scl=Pin(22))
adc = ADS1115(i2c, address=73, gain=1)

in1 = Pin(12, mode=Pin.OUT)
in2 = Pin(14, mode=Pin.OUT)
in3 = Pin(27, mode=Pin.OUT)
in4 = Pin(26, mode=Pin.OUT)

# while True:
#     print(adc.raw_to_v(adc.read(0,1)) / 3.3 * 100)
#     sleep(0.1)


try:
  import usocket as socket
except:
  import socket

import network

import esp
esp.osdebug(None)

import gc
gc.collect()


def main():
    ssid = 'MicroPython-AP'
    password = '123456789'

    ap = network.WLAN(network.AP_IF)
    ap.active(True)
    ap.config(essid=ssid, password=password)

    while ap.active() == False:
        pass

    print('Connection successful')
    print(ap.ifconfig())

    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.bind(('', 80))
    s.listen(5)

    conn: socket.socket
    conn, addr = s.accept()
    print('Got a connection from %s' % str(addr))
    while True:
        conn.settimeout(1.0)
        try:
            request = conn.recv(1024)
            print('Content = %s' % str(request))
            if "byebye" in request:
                break
        except (OSError, socket.TimeoutError) as e:
            print('Timeout')
        encoder = adc.raw_to_v(adc.read(0,1)) / 3.3 * 100
        response = str(encoder)
        conn.send(response)
    conn.close()



from machine import Pin, PWM, I2C

from ads1x15 import ADS1115

from utime import sleep, ticks_ms, ticks_diff

try:
  import usocket as socket
except:
  import socket

import uselect

import network

import esp
esp.osdebug(None)

import gc
gc.collect()


i2c = I2C(0, sda=Pin(21), scl=Pin(22))
adc = ADS1115(i2c, address=73, gain=1)

ena = Pin(12)
pwm_ena = PWM(ena, freq=1000)
pwm_ena.duty_u16(0)

in1 = Pin(14, mode=Pin.OUT)
in2 = Pin(27, mode=Pin.OUT)
in3 = Pin(26, mode=Pin.OUT)
in4 = Pin(25, mode=Pin.OUT)

enb = Pin(33, mode=Pin.OUT)
pwm_enb = PWM(enb, freq=1000)
pwm_ena.duty_u16(0)


BODY_CHANNELS = [1, 2]
REVERSE_ENCODERS = [False, False]
HALF_RANGE_DEG = 70

def encoder_angles(adc: ADS1115) -> list[float]:
    output = []
    for channel, reverse in zip(BODY_CHANNELS, REVERSE_ENCODERS):
        proportion = adc.raw_to_v(adc.read(0, channel)) / 3.3
        angle = (proportion-0.5) * 2 * HALF_RANGE_DEG
        if reverse:
            angle = -angle
        output.append(angle)
    return output


PRIMEMOVER_TRACK_HALF = 0.1 / 2  # metres
PRIMEMOVER_WHEEL_RADIUS = 0.04  # metres
SPEED_CORRECTION = 1
MAX_WHEEL_ANGVEL = 2*3.1416*(204/60)

REVERSE_RIGHT_WHEEL = False
REVERSE_LEFT_WHEEL = False


def drive(v: float, omega: float):
    right_wheel_speed = (v + omega * PRIMEMOVER_TRACK_HALF) / PRIMEMOVER_WHEEL_RADIUS
    left_wheel_speed = (v - omega * PRIMEMOVER_TRACK_HALF) / PRIMEMOVER_WHEEL_RADIUS
    right_wheel_speed *= SPEED_CORRECTION
    left_wheel_speed *= SPEED_CORRECTION
    right_proportion = right_wheel_speed / MAX_WHEEL_ANGVEL
    left_proportion = left_wheel_speed / MAX_WHEEL_ANGVEL
    
    if (right_proportion >= 0) ^ REVERSE_RIGHT_WHEEL:
        in1.on()
        in2.off()
    else:
        in1.off()
        in2.on()
    pwm_ena.duty_u16(min(65535, int(65535*abs(right_proportion))))

    if (left_proportion >= 0) ^ REVERSE_LEFT_WHEEL:
        in3.on()
        in4.off()
    else:
        in3.off()
        in4.on()
    pwm_enb.duty_u16(min(65535, int(65535*abs(left_proportion))))


def main():
    # ssid = 'MicroPython-AP'
    # password = '123456789'

    # ap = network.WLAN(network.AP_IF)
    # ap.active(True)
    # ap.config(essid=ssid, password=password)

    # while not ap.active():
    #     pass

    # print('Connection successful')
    # print(ap.ifconfig())

    ssid = "Sjdhwusbehd"
    password = "mrbd7016"

    wifi = network.WLAN(network.STA_IF)
    wifi.active(True)
    wifi.connect(ssid, password)
    
    while not wifi.isconnected():
        pass

    print('Connection successful')
    print(wifi.ifconfig())

    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.bind(('', 80))
    s.listen(5)

    timeout_ms = 1

    last_time = ticks_ms()

    while True:
        conn: socket.socket
        conn, addr = s.accept()
        poller = uselect.poll()
        poller.register(conn, uselect.POLLIN)
        print('Got a connection from %s' % str(addr))
        while True:
            try:
                res = poller.poll(timeout_ms)
                # res = True
                if res:
                    request = conn.recv(1024).decode()
                    print('Content =', str(request))
                    packets = request.split("[")
                    for packet in packets:
                        if len(packet) == 0:
                            continue
                        if packet[-1] != "]":
                            continue
                        try:
                            values = [float(x) for x in packet[:-1].split(",")]
                        except ValueError:
                            continue
                        if len(values) != 2:
                            continue
                        v, omega = values
                        drive(v, omega)
            except OSError as e:
                if e.errno == 116:
                    # Timed out
                    continue
                else:
                    print("Disconnected:", e)
                    break

            # now_time = ticks_ms()
            # if ticks_diff(now_time, last_time) >= 100:
            #     last_time = now_time
            #     try:
            #         response = str(encoder_angles(adc))
            #         conn.send(response)
            #     except OSError as e:
            #         print("ADC Failed:", e)
            
        conn.close()

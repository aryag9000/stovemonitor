from machine import Pin, I2C, ADC, Timer
from hcsr04 import HCSR04
import math
import network
import socket
import time
from time import sleep, ticks_ms

# Wi-Fi credentials
ssid = 'Arya Ghorpade'
password = 'ghorpade wifi' #replace with your own wifi network and password

# Pins and peripherals
led = Pin("LED", Pin.OUT)
i2c = I2C(0, scl=Pin(21), sda=Pin(20), freq=400000)
sensor = HCSR04(trigger_pin=2, echo_pin=3, echo_timeout_us=1000)
temp = ADC(4)
mpu6050_addr = 0x68
ACCEL_XOUT_H = 0x3B
i2c.writeto_mem(mpu6050_addr, 0x6B, b'\x00')  # Wake up the MPU6050

# Global variables
stove_on_time = 0
stove_warning = "No warnings"
temp_warning = ""
distance = 0
angle = 0
temperature = 0

# Timer and state flags
timers = {}
stove_check_flag = False
distance_flag = False
temperature_flag = False

# Functions
def read_temperature():
    adc_value = temp.read_u16()
    volt = (3.3 / 65535) * adc_value
    return round(27 - (volt - 0.706) / 0.001721, 1)

def read_raw_data(register):
    high_byte = i2c.readfrom_mem(mpu6050_addr, register, 1)[0]
    low_byte = i2c.readfrom_mem(mpu6050_addr, register + 1, 1)[0]
    value = (high_byte << 8) | low_byte
    return value - 65536 if value > 32768 else value

def calculate_angles():
    AcX = read_raw_data(ACCEL_XOUT_H)
    AcY = read_raw_data(ACCEL_XOUT_H + 2)
    AcZ = read_raw_data(ACCEL_XOUT_H + 4)
    AcX_norm, AcY_norm, AcZ_norm = AcX / 16384.0, AcY / 16384.0, AcZ / 16384.0
    return math.degrees(math.atan2(AcY_norm, AcX_norm))

def read_distance():
    global distance
    distance = sensor.distance_cm()

def update_sensor_data(timer):
    global angle, temperature
    angle = calculate_angles()
    temperature = read_temperature()

def check_stove_status(timer):
    global stove_on_time, stove_warning, temp_warning, led
    if angle < 80 or angle > 91:
        if stove_on_time == 0:
            stove_on_time = time.ticks_ms()
    else:
        stove_on_time = 0
        led.off()

    # Update warnings
    elapsed_time = (ticks_ms() - stove_on_time) / 1000 if stove_on_time > 0 else 0
    if stove_on_time == 0:
        stove_warning = "Stove is off."
        led.off()
    elif elapsed_time > 20 and distance < 2:
        stove_warning = "WARNING!! Stove is on and no one is there."
        led.on()
    else:
        led.off()
        if distance < 2 :
            stove_warning = "Stove is on but under normal conditions."
        else:
            stove_warning= "Stove is on, someone is present."
            stove_on_time=0

    temp_warning = "WARNING!! Stove is really hot!" if temperature > 27 else "Stove temp is not too high."

def connect():
    wlan = network.WLAN(network.STA_IF)
    wlan.active(True)
    wlan.connect(ssid, password)
    while not wlan.isconnected():
        pass
    return wlan.ifconfig()[0]

def open_socket(ip):
    address = (ip, 80)
    connection = socket.socket()
    connection.bind(address)
    connection.listen(1)
    return connection

def webpage():
    html = f"""
            <!DOCTYPE html>
            <html>
            <head>
                <meta http-equiv="refresh" content="5">
            </head>
            <body>
            <h1>Stove Monitor</h1>
            <p>{stove_warning}</p>
            <p>{temp_warning}</p>
            </body>
            </html>
            """
    return html

def serve(connection):
    while True:
        client = connection.accept()[0]
        client.recv(1024)  # Request ignored for simplicity
        client.send(webpage())
        client.close()

# Setup timers
timers['sensor'] = Timer(period=1000, mode=Timer.PERIODIC, callback=update_sensor_data)  # 1-second interval
timers['stove'] = Timer(period=500, mode=Timer.PERIODIC, callback=check_stove_status)   # 0.5-second interval
timers['distance'] = Timer(period=500, mode=Timer.PERIODIC, callback=lambda t: read_distance())

# Main execution
try:
    ip = connect()
    connection = open_socket(ip)
    serve(connection)
except KeyboardInterrupt:
    for t in timers.values():
        t.deinit()
    machine.reset()

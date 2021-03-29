import serial
import adafruit_bno055
import time
uart = serial.Serial("/dev/ttyUSB1")
sensor = adafruit_bno055.BNO055_UART(uart)

while True:
    print(f'temp: {sensor.temperature}, euler: {sensor.euler}, gravity: {sensor.gravity}')
    time.sleep(0.05)
    
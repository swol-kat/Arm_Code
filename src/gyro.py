import adafruit_bno055
from busio import I2C
from board import SDA, SCL

i2c = I2C(SCL, SDA)
sensor = adafruit_bno055.BNO055_I2C(i2c)

while True:
    print(f'temp: {sensor.temperature}, euler: {sensor.euler}, gravity: {sensor.gravity}')
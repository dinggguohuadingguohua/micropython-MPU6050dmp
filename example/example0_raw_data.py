"""
MPU6050 Micropython I2C Class Support ESP32 and ESP8266, etc
Code based on 
1 - MPU6050 I2C device class
by Jeff Rowberg Geir Istad  2015  
2 - MPU6050 Python I2C Class 
by Majid Alekasir 2024 MAY
============================================
MIT license
Copyright (c) 2024 yanhuafeng
Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:
The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.
THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
===============================================
"""
from MPU6050 import MPU6050
from machine import I2C,Pin

i2c = I2C(scl=Pin(6), sda=Pin(7))
device_address = 0x68
freq_divider = 0x04 # freq = 200Hz / (1 + value)  推荐 >=4

# Make an MPU6050
mpu = MPU6050(i2c, device_address, freq_divider)

# Initiate your DMP
mpu.dmp_initialize()
mpu.set_DMP_enabled(True)

packet_size = mpu.DMP_get_FIFO_packet_size()
FIFO_buffer = [0]*64

g = 9.8 # gravity acceleration (m/s^2)

while True: # infinite loop
    while not mpu.isreadyFIFO(packet_size): # Check if FIFO data are ready to use...
        pass

    # DMP acceleration (less noisy acceleration - based on fusion)
    FIFO_buffer = mpu.get_FIFO_bytes(packet_size) 
    dmp_x,dmp_y,dmp_z = mpu.DMP_get_acceleration_int16(FIFO_buffer)
    Ax_dmp = dmp_x * 2*g / 2**15 * 2 # 单位换算 [-2g, +2g]. [-2^15, +2^15]
    Ay_dmp = dmp_y * 2*g / 2**15 * 2
    Az_dmp = dmp_z * 2*g / 2**15 * 2


    # raw acceleration 
    accl_x,accl_y,accl_z = mpu.acceleration
    Ax = accl_x * 2*g / 2**15 # 单位换算 [-2g, +2g]. [-2^15, +2^15]
    Ay = accl_y * 2*g / 2**15
    Az = accl_z * 2*g / 2**15
    
    
    # raw gyro (full range: [-250, +250]) (unit: degree / second)
    gyro_x,gyro_y,gyro_z = mpu.rotation
    Gx = gyro_x * 250 / 2**15 # 单位换算 [-250, +250]. [-2^15, +2^15]
    Gy = gyro_y * 250 / 2**15
    Gz = gyro_z * 250 / 2**15


    print("AcX: {}, AcY: {}, AcZ: {}".format(Ax, Ay, Az))

"""
MPU6050 Micropython I2C Class Support ESP32 and ESP8266, etc
Code based on 
1 - I2Cdev library collection  MPU6050 I2C device class
by Jeff Rowberg Geir Istad  2015
2 - MPU6050 Python I2C Class 
by Majid Alekasir 2024
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
mpu = MPU6050(i2c, device_address)

# 执行一次即可
# mpu.dmp_initialize()
# mpu.set_DMP_enabled(True)

while True: # infinite loop
    while not mpu.isreadyFIFO(): # 等待数据
        pass
        
    FIFO_buffer = mpu.get_FIFO_bytes()
    
    qw,qx,qy,qz = mpu.DMP_get_quaternion(FIFO_buffer)
    roll, pitch, yaw = mpu.DMP_get_euler_roll_pitch_yaw(qw,qx,qy,qz)
    print("roll: {}, pitch: {}, yaw: {}".format(roll, pitch, yaw))
    
    ax,ay,az = mpu.DMP_get_acceleration_int16(FIFO_buffer)
    grav_x,grav_y,grav_z = mpu.DMP_get_gravity(qw,qx,qy,qz)
    acc_x,acc_y,acc_z = mpu.DMP_get_linear_accel(ax, ay, az ,grav_x,grav_y,grav_z ) # 矫正后加速度
    
    

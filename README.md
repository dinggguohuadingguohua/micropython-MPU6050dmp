# micropython-MPU6050dmp
micropython-MPU6050dmp   
为了解决 micropython 难以使用 dmp 的问题，推出此项目，充分利用硬件资源可以大大减轻工作量

## 使用介绍
> 须上传到 flash 中的文件  
> - dmp.bin  
> - MPU6050.py 

基础配置
```py
from MPU6050 import MPU6050
from machine import I2C,Pin
i2c = I2C(scl=Pin(6), sda=Pin(7)) 
device_address = 0x68     # i2c通讯地址
mpu = MPU6050(i2c, device_address)

mpu.calibrate(256, 2)     # 运行一次即可,可选，矫正角速度偏移

mpu.dmp_initialize()      # 运行一次即可，必须，下载 dmp
mpu.set_DMP_enabled(True) # 运行一次即可，必须，打开 dmp
```

读取原始数据
```py
accl_x,accl_y,accl_z = mpu.acceleration
gyro_x,gyro_y,gyro_z = mpu.rotation
```

读取dmp数据
```py
FIFO = mpu.get_FIFO_bytes()
qw,qx,qy,qz = mpu.DMP_get_quaternion(FIFO) # /2**15
qw,qx,qy,qz = mpu.DMP_get_quaternion_int16(FIFO)
ax,ay,az = mpu.DMP_get_acceleration_int16(FIFO)
```

数学计算
```py
roll, pitch, yaw = mpu.get_roll_pitch_yaw( w,x,y,z) # 弧度
roll, pitch, yaw = mpu.get_euler_roll_pitch_yaw( w,x,y,z) # 角度
psi, theta, phi = mpu.get_euler( w,x,y,z) # 角度
Ax_linear, Ay_linear, Az_linear = mpu.get_accel_InWorld( acc_list , quat_list) # 加速度
```

可以直接运行 example 中的案例

推荐使用 `mpu.isok` 来确定 iic 是否正常工作

## 对于小白
使用 iic.readfrom_mem 和 iic.writeto_mem 来读取/配置寄存器     
绝大多数寄存器查阅 https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Register-Map1.pdf

### FIFO格式
```
     * ================================================================================================ *
     | Default MotionApps v2.0 42-byte FIFO packet structure:                                           |
     |                                                                                                  |
     | [QUAT W][      ][QUAT X][      ][QUAT Y][      ][QUAT Z][      ][GYRO X][      ][GYRO Y][      ] |
     |   0   1   2   3   4   5   6   7   8   9  10  11  12  13  14  15  16  17  18  19  20  21  22  23  |
     |                                                                                                  |
     | [GYRO Z][      ][ACC X ][      ][ACC Y ][      ][ACC Z ][      ][      ]                         |
     |  24  25  26  27  28  29  30  31  32  33  34  35  36  37  38  39  40  41                          |
     * ================================================================================================ *
```


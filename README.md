# micropython-MPU6050dmp
micropython-MPU6050dmp
使用方法参考 example 中的文件

## 使用介绍
> 须上传到 flash 中的文件  
- dmp.bin
- MPU6050.py 

上传后可以运行 examples 中的案例

这里以 mpu 举例  
推荐使用 `mpu.isok` 来确定 iic 是否正常工作

## 对于小白
使用 iic.readfrom_mem 和 iic.writeto_mem 来读取/配置寄存器   
绝大多数寄存器查阅 https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Register-Map1.pdf

## FIFO格式
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


import math,struct,time
from micropython import const

# From MPU6050.h
# address pin low (GND), default
_ADDRESS_AD0_LOW = const(0x68)
# address pin high (VCC)
_ADDRESS_AD0_HIGH = const(0x69)
_DEFAULT_ADDRESS = const(_ADDRESS_AD0_LOW)

# [7] PWR_MODE, [6:1] XG_OFFS_TC, [0] OTP_BNK_VLD
_RA_XG_OFFS_TC = const(0x00)
# [7] PWR_MODE, [6:1] YG_OFFS_TC, [0] OTP_BNK_VLD
_RA_YG_OFFS_TC = const(0x01)
# [7] PWR_MODE, [6:1] ZG_OFFS_TC, [0] OTP_BNK_VLD
_RA_ZG_OFFS_TC = const(0x02)
# [7:0] X_FINE_GAIN
_RA_X_FINE_GAIN = const(0x03)
# [7:0] Y_FINE_GAIN
_RA_Y_FINE_GAIN = const(0x04)
# [7:0] Z_FINE_GAIN
_RA_Z_FINE_GAIN = const(0x05)
# [15:0] XA_OFFS
_RA_XA_OFFS_H = const(0x06)
_RA_XA_OFFS_L_TC = const(0x07)
# [15:0] YA_OFFS
_RA_YA_OFFS_H = const(0x08)
_RA_YA_OFFS_L_TC = const(0x09)
# [15:0] ZA_OFFS
_RA_ZA_OFFS_H = const(0x0A)
_RA_ZA_OFFS_L_TC = const(0x0B)
# [15:0] XG_OFFS_USR
_RA_XG_OFFS_USRH = const(0x13)
_RA_XG_OFFS_USRL = const(0x14)
# [15:0] YG_OFFS_USR
_RA_YG_OFFS_USRH = const(0x15)
_RA_YG_OFFS_USRL = const(0x16)
# [15:0] ZG_OFFS_USR
_RA_ZG_OFFS_USRH = const(0x17)
_RA_ZG_OFFS_USRL = const(0x18)
_RA_SMPLRT_DIV = const(0x19)
_RA_CONFIG = const(0x1A)
_RA_GYRO_CONFIG = const(0x1B)
_RA_ACCEL_CONFIG = const(0x1C)
_RA_FF_THR = const(0x1D)
_RA_FF_DUR = const(0x1E)
_RA_MOT_THR = const(0x1F)
_RA_MOT_DUR = const(0x20)
_RA_ZRMOT_THR = const(0x21)
_RA_ZRMOT_DUR = const(0x22)
_RA_FIFO_EN = const(0x23)
_RA_I2C_MST_CTRL = const(0x24)
_RA_I2C_SLV0_ADDR = const(0x25)
# _RA_I2C_SLV0_REG = const(0x26)
# _RA_I2C_SLV0_CTRL = const(0x27)
# _RA_I2C_SLV1_ADDR = const(0x28)
# _RA_I2C_SLV1_REG = const(0x29)
# _RA_I2C_SLV1_CTRL = const(0x2A)
# _RA_I2C_SLV2_ADDR = const(0x2B)
# _RA_I2C_SLV2_REG = const(0x2C)
# _RA_I2C_SLV2_CTRL = const(0x2D)
# _RA_I2C_SLV3_ADDR = const(0x2E)
# _RA_I2C_SLV3_REG = const(0x2F)
# _RA_I2C_SLV3_CTRL = const(0x30)
# _RA_I2C_SLV4_ADDR = const(0x31)
# _RA_I2C_SLV4_REG = const(0x32)
# _RA_I2C_SLV4_DO = const(0x33)
# _RA_I2C_SLV4_CTRL = const(0x34)
# _RA_I2C_SLV4_DI = const(0x35)
# _RA_I2C_MST_STATUS = const(0x36)
# _RA_INT_PIN_CFG = const(0x37)
_RA_INT_ENABLE = const(0x38)
_RA_DMP_INT_STATUS = const(0x39)
_RA_INT_STATUS = const(0x3A)
_RA_ACCEL_XOUT_H = const(0x3B)
# _RA_ACCEL_XOUT_L = const(0x3C)
# _RA_ACCEL_YOUT_H = const(0x3D)
# _RA_ACCEL_YOUT_L = const(0x3E)
# _RA_ACCEL_ZOUT_H = const(0x3F)
# _RA_ACCEL_ZOUT_L = const(0x40)
# _RA_TEMP_OUT_H = const(0x41)
# _RA_TEMP_OUT_L = const(0x42)
_RA_GYRO_XOUT_H = const(0x43)
# _RA_GYRO_XOUT_L = const(0x44)
# _RA_GYRO_YOUT_H = const(0x45)
# _RA_GYRO_YOUT_L = const(0x46)
# _RA_GYRO_ZOUT_H = const(0x47)
# _RA_GYRO_ZOUT_L = const(0x48)
# _RA_EXT_SENS_DATA_00 = const(0x49)
# _RA_EXT_SENS_DATA_01 = const(0x4A)
# _RA_EXT_SENS_DATA_02 = const(0x4B)
# _RA_EXT_SENS_DATA_03 = const(0x4C)
# _RA_EXT_SENS_DATA_04 = const(0x4D)
# _RA_EXT_SENS_DATA_05 = const(0x4E)
# _RA_EXT_SENS_DATA_06 = const(0x4F)
# _RA_EXT_SENS_DATA_07 = const(0x50)
# _RA_EXT_SENS_DATA_08 = const(0x51)
# _RA_EXT_SENS_DATA_09 = const(0x52)
# _RA_EXT_SENS_DATA_10 = const(0x53)
# _RA_EXT_SENS_DATA_11 = const(0x54)
# _RA_EXT_SENS_DATA_12 = const(0x55)
# _RA_EXT_SENS_DATA_13 = const(0x56)
# _RA_EXT_SENS_DATA_14 = const(0x57)
# _RA_EXT_SENS_DATA_15 = const(0x58)
# _RA_EXT_SENS_DATA_16 = const(0x59)
# _RA_EXT_SENS_DATA_17 = const(0x5A)
# _RA_EXT_SENS_DATA_18 = const(0x5B)
# _RA_EXT_SENS_DATA_19 = const(0x5C)
# _RA_EXT_SENS_DATA_20 = const(0x5D)
# _RA_EXT_SENS_DATA_21 = const(0x5E)
# _RA_EXT_SENS_DATA_22 = const(0x5F)
# _RA_EXT_SENS_DATA_23 = const(0x60)
# _RA_MOT_DETECT_STATUS = const(0x61)
# _RA_I2C_SLV0_DO = const(0x63)
# _RA_I2C_SLV1_DO = const(0x64)
# _RA_I2C_SLV2_DO = const(0x65)
# _RA_I2C_SLV3_DO = const(0x66)
# _RA_I2C_MST_DELAY_CTRL = const(0x67)
# _RA_SIGNAL_PATH_RESET = const(0x68)
# _RA_MOT_DETECT_CTRL = const(0x69)
_RA_USER_CTRL = const(0x6A)
_RA_PWR_MGMT_1 = const(0x6B)
_RA_PWR_MGMT_2 = const(0x6C)
_RA_BANK_SEL = const(0x6D)
_RA_MEM_START_ADDR = const(0x6E)
_RA_MEM_R_W = const(0x6F)
_RA_DMP_CFG_1 = const(0x70)
_RA_DMP_CFG_2 = const(0x71)
_RA_FIFO_COUNTH = const(0x72)
_RA_FIFO_COUNTL = const(0x73)
_RA_FIFO_R_W = const(0x74)
_RA_WHO_AM_I = const(0x75)

_TC_PWR_MODE_BIT = const(7)
_TC_OFFSET_BIT = const(6)
_TC_OFFSET_LENGTH = const(6)
_TC_OTP_BNK_VLD_BIT = const(0)

_VDDIO_LEVEL_VLOGIC = const(0)
_VDDIO_LEVEL_VDD = const(1)

_CFG_EXT_SYNC_SET_BIT = const(5)
_CFG_EXT_SYNC_SET_LENGTH = const(3)
_CFG_DLPF_CFG_BIT = const(2)
_CFG_DLPF_CFG_LENGTH = const(3)

_EXT_SYNC_DISABLED = const(0x0)
_EXT_SYNC_TEMP_OUT_L = const(0x1)
# _EXT_SYNC_GYRO_XOUT_L = const(0x2)
# _EXT_SYNC_GYRO_YOUT_L = const(0x3)
# _EXT_SYNC_GYRO_ZOUT_L = const(0x4)
# _EXT_SYNC_ACCEL_XOUT_L = const(0x5)
# _EXT_SYNC_ACCEL_YOUT_L = const(0x6)
# _EXT_SYNC_ACCEL_ZOUT_L = const(0x7)

# _DLPF_BW_256 = const(0x00)
# _DLPF_BW_188 = const(0x01)
# _DLPF_BW_98 = const(0x02)
_DLPF_BW_42 = const(0x03)
# _DLPF_BW_20 = const(0x04)
# _DLPF_BW_10 = const(0x05)
# _DLPF_BW_5 = const(0x06)

_GCONFIG_FS_SEL_BIT = const(4)
_GCONFIG_FS_SEL_LENGTH = const(2)

_GYRO_FS_250 = const(0x00)
_GYRO_FS_500 = const(0x01)
_GYRO_FS_1000 = const(0x02)
_GYRO_FS_2000 = const(0x03)

_ACONFIG_XA_ST_BIT = const(7)
_ACONFIG_YA_ST_BIT = const(6)
_ACONFIG_ZA_ST_BIT = const(5)
_ACONFIG_AFS_SEL_BIT = const(4)
_ACONFIG_AFS_SEL_LENGTH = const(2)
_ACONFIG_ACCEL_HPF_BIT = const(2)
_ACONFIG_ACCEL_HPF_LENGTH = const(3)

_ACCEL_FS_2 = const(0x00)
_ACCEL_FS_4 = const(0x01)
_ACCEL_FS_8 = const(0x02)
_ACCEL_FS_16 = const(0x03)

# _DHPF_RESET = const(0x00)
# _DHPF_5 = const(0x01)
# _DHPF_2P5 = const(0x02)
# _DHPF_1P25 = const(0x03)
# _DHPF_0P63 = const(0x04)
# _DHPF_HOLD = const(0x07)

# _TEMP_FIFO_EN_BIT = const(7)
# _XG_FIFO_EN_BIT = const(6)
# _YG_FIFO_EN_BIT = const(5)
# _ZG_FIFO_EN_BIT = const(4)
# _ACCEL_FIFO_EN_BIT = const(3)
# _SLV2_FIFO_EN_BIT = const(2)
# _SLV1_FIFO_EN_BIT = const(1)
# _SLV0_FIFO_EN_BIT = const(0)

# _MULT_MST_EN_BIT = const(7)
# _WAIT_FOR_ES_BIT = const(6)
# _SLV_3_FIFO_EN_BIT = const(5)
# _I2C_MST_P_NSR_BIT = const(4)
# _I2C_MST_CLK_BIT = const(3)
# _I2C_MST_CLK_LENGTH = const(4)

# _CLOCK_DIV_348 = const(0x0)
# _CLOCK_DIV_333 = const(0x1)
# _CLOCK_DIV_320 = const(0x2)
# _CLOCK_DIV_308 = const(0x3)
# _CLOCK_DIV_296 = const(0x4)
# _CLOCK_DIV_286 = const(0x5)
# _CLOCK_DIV_276 = const(0x6)
# _CLOCK_DIV_267 = const(0x7)
# _CLOCK_DIV_258 = const(0x8)
# _CLOCK_DIV_500 = const(0x9)
# _CLOCK_DIV_471 = const(0xA)
# _CLOCK_DIV_444 = const(0xB)
# _CLOCK_DIV_421 = const(0xC)
# _CLOCK_DIV_400 = const(0xD)
# _CLOCK_DIV_381 = const(0xE)
# _CLOCK_DIV_364 = const(0xF)

# _I2C_SLV_RW_BIT = const(7)
# _I2C_SLV_ADDR_BIT = const(6)
# _I2C_SLV_ADDR_LENGTH = const(7)
# _I2C_SLV_EN_BIT = const(7)
# _I2C_SLV_BYTE_SW_BIT = const(6)
# _I2C_SLV_REG_DIS_BIT = const(5)
# _I2C_SLV_GRP_BIT = const(4)
# _I2C_SLV_LEN_BIT = const(3)
# _I2C_SLV_LEN_LENGTH = const(4)

# _I2C_SLV4_RW_BIT = const(7)
# _I2C_SLV4_ADDR_BIT = const(6)
# _I2C_SLV4_ADDR_LENGTH = const(7)
# _I2C_SLV4_EN_BIT = const(7)
# _I2C_SLV4_INT_EN_BIT = const(6)
# _I2C_SLV4_REG_DIS_BIT = const(5)
# _I2C_SLV4_MST_DLY_BIT = const(4)
# _I2C_SLV4_MST_DLY_LENGTH = const(5)

# _MST_PASS_THROUGH_BIT = const(7)
# _MST_I2C_SLV4_DONE_BIT = const(6)
# _MST_I2C_LOST_ARB_BIT = const(5)
# _MST_I2C_SLV4_NACK_BIT = const(4)
# _MST_I2C_SLV3_NACK_BIT = const(3)
# _MST_I2C_SLV2_NACK_BIT = const(2)
# _MST_I2C_SLV1_NACK_BIT = const(1)
# _MST_I2C_SLV0_NACK_BIT = const(0)

# _INTCFG_INT_LEVEL_BIT = const(7)
# _INTCFG_INT_OPEN_BIT = const(6)
# _INTCFG_LATCH_INT_EN_BIT = const(5)
# _INTCFG_INT_RD_CLEAR_BIT = const(4)
# _INTCFG_FSYNC_INT_LEVEL_BIT = const(3)
# _INTCFG_FSYNC_INT_EN_BIT = const(2)
# _INTCFG_I2C_BYPASS_EN_BIT = const(1)
# _INTCFG_CLKOUT_EN_BIT = const(0)

# _INTMODE_ACTIVEHIGH = const(0x00)
# _INTMODE_ACTIVELOW = const(0x01)

# _INTDRV_PUSHPULL = const(0x00)
# _INTDRV_OPENDRAIN = const(0x01)

# _INTLATCH_50USPULSE = const(0x00)
# _INTLATCH_WAITCLEAR = const(0x01)

# _INTCLEAR_STATUSREAD = const(0x00)
# _INTCLEAR_ANYREAD = const(0x01)

# _INTERRUPT_FF_BIT = const(7)
# _INTERRUPT_MOT_BIT = const(6)
# _INTERRUPT_ZMOT_BIT = const(5)
# _INTERRUPT_FIFO_OFLOW_BIT = const(4)
# _INTERRUPT_I2C_MST_INT_BIT = const(3)
# _INTERRUPT_PLL_RDY_INT_BIT = const(2)
# _INTERRUPT_DMP_INT_BIT = const(1)
# _INTERRUPT_DATA_RDY_BIT = const(0)

# TODO: figure out what these actually do
# UMPL source code is not very obivous
# _DMPINT_5_BIT = const(5)
# _DMPINT_4_BIT = const(4)
# _DMPINT_3_BIT = const(3)
# _DMPINT_2_BIT = const(2)
# _DMPINT_1_BIT = const(1)
# _DMPINT_0_BIT = const(0)

# _MOTION_MOT_XNEG_BIT = const(7)
# _MOTION_MOT_XPOS_BIT = const(6)
# _MOTION_MOT_YNEG_BIT = const(5)
# _MOTION_MOT_YPOS_BIT = const(4)
# _MOTION_MOT_ZNEG_BIT = const(3)
# _MOTION_MOT_ZPOS_BIT = const(2)
# _MOTION_MOT_ZRMOT_BIT = const(0)

# _DELAYCTRL_DELAY_ES_SHADOW_BIT = const(7)
# _DELAYCTRL_I2C_SLV4_DLY_EN_BIT = const(4)
# _DELAYCTRL_I2C_SLV3_DLY_EN_BIT = const(3)
# _DELAYCTRL_I2C_SLV2_DLY_EN_BIT = const(2)
# _DELAYCTRL_I2C_SLV1_DLY_EN_BIT = const(1)
# _DELAYCTRL_I2C_SLV0_DLY_EN_BIT = const(0)

# _PATHRESET_GYRO_RESET_BIT = const(2)
# _PATHRESET_ACCEL_RESET_BIT = const(1)
# _PATHRESET_TEMP_RESET_BIT = const(0)

# _DETECT_ACCEL_ON_DELAY_BIT = const(5)
# _DETECT_ACCEL_ON_DELAY_LENGTH = const(2)
# _DETECT_FF_COUNT_BIT = const(3)
# _DETECT_FF_COUNT_LENGTH = const(2)
# _DETECT_MOT_COUNT_BIT = const(1)
# _DETECT_MOT_COUNT_LENGTH = const(2)

# _DETECT_DECREMENT_RESET = const(0x0)
# _DETECT_DECREMENT_1 = const(0x1)
# _DETECT_DECREMENT_2 = const(0x2)
# _DETECT_DECREMENT_4 = const(0x3)

_USERCTRL_DMP_EN_BIT = const(7)
_USERCTRL_FIFO_EN_BIT = const(6)
_USERCTRL_I2C_MST_EN_BIT = const(5)
_USERCTRL_I2C_IF_DIS_BIT = const(4)
_USERCTRL_DMP_RESET_BIT = const(3)
_USERCTRL_FIFO_RESET_BIT = const(2)
_USERCTRL_I2C_MST_RESET_BIT = const(1)
_USERCTRL_SIG_COND_RESET_BIT = const(0)

_PWR1_DEVICE_RESET_BIT = const(7)
_PWR1_SLEEP_BIT = const(6)
_PWR1_CYCLE_BIT = const(5)
_PWR1_TEMP_DIS_BIT = const(3)
_PWR1_CLKSEL_BIT = const(2)
_PWR1_CLKSEL_LENGTH = const(3)

_CLOCK_INTERNAL = const(0x00)
_CLOCK_PLL_XGYRO = const(0x01)
_CLOCK_PLL_YGYRO = const(0x02)
_CLOCK_PLL_ZGYRO = const(0x03)
# _CLOCK_PLL_EXT32K = const(0x04)
# _CLOCK_PLL_EXT19M = const(0x05)
# _CLOCK_KEEP_RESET = const(0x07)

# _PWR2_LP_WAKE_CTRL_BIT = const(7)
# _PWR2_LP_WAKE_CTRL_LENGTH = const(2)
# _PWR2_STBY_XA_BIT = const(5)
# _PWR2_STBY_YA_BIT = const(4)
# _PWR2_STBY_ZA_BIT = const(3)
# _PWR2_STBY_XG_BIT = const(2)
# _PWR2_STBY_YG_BIT = const(1)
# _PWR2_STBY_ZG_BIT = const(0)

# _WAKE_FREQ_1P25 = const(0x0)
# _WAKE_FREQ_2P5 = const(0x1)
# _WAKE_FREQ_5 = const(0x2)
# _WAKE_FREQ_10 = const(0x3)

# _BANKSEL_PRFTCH_EN_BIT = const(6)
# _BANKSEL_CFG_USER_BANK_BIT = const(5)
# _BANKSEL_MEM_SEL_BIT = const(4)
# _BANKSEL_MEM_SEL_LENGTH = const(5)

# _WHO_AM_I_BIT = const(6)
# _WHO_AM_I_LENGTH = const(6)

# _DMP_MEMORY_BANKS = const(8)
# _DMP_MEMORY_BANK_SIZE = const(256)
# _DMP_MEMORY_CHUNK_SIZE = const(16)

# From 6Axis_MotionApps20.h
_DMP_CODE_SIZE = const(1929)
_DMP_CONFIG_SIZE = const(192)
_DMP_UPDATES_SIZE = const(47)
_dmp_Memory = const(0)
_dmp_Config = const(1929)
_dmp_Updates = const(2121) # 1929+192=2121
    
    
class i2c_func:
    def __init__(self,  i2c):
        self.iic = i2c
    def read_byte_data(self,__dev_id, reg_add):
        return self.iic.readfrom_mem(__dev_id, reg_add, 1)[0]
    def read_i2c_block_data(self,__dev_id , reg_add , length):
        return self.iic.readfrom_mem(__dev_id, reg_add, length)
    def write_byte_data(self,__dev_id,reg_add, value):
        return self.iic.writeto_mem (__dev_id, reg_add, bytes([value])) 
    

class MPU6050:
    __buffer = [0] * 14
    __debug = False
    __DMP_packet_size = 0
    __dev_id = 0
    __bus = None

    def __init__(self,  i2c, a_address=_DEFAULT_ADDRESS, freq_divider=4,
                 a_xAOff=0, a_yAOff=0, a_zAOff=0, a_xGOff=0,
                 a_yGOff=0, a_zGOff=0, a_debug=False):

        ### Define the divider of the DMP frequency
        self.freq_divider = freq_divider

        self.__dev_id = a_address
        # Connect to num 1 SMBus
        self.__bus = i2c_func(i2c)
        # Set clock source to gyro
        self.set_clock_source(_CLOCK_PLL_XGYRO)
        # Set accelerometer range
        self.set_full_scale_accel_range(_ACCEL_FS_2)
        # Set gyro range
        self.set_full_scale_gyro_range(_GYRO_FS_250)
        # Take the MPU out of time.sleep mode
        self.wake_up()
        # Set offsets
        if a_xAOff:
            self.set_x_accel_offset(a_xAOff)
        if a_yAOff:
            self.set_y_accel_offset(a_yAOff)
        if a_zAOff:
            self.set_z_accel_offset(a_zAOff)
        if a_xGOff:
            self.set_x_gyro_offset(a_xGOff)
        if a_yGOff:
            self.set_y_gyro_offset(a_yGOff)
        if a_zGOff:
            self.set_z_gyro_offset(a_zGOff)
        self.__debug = a_debug

    @property
    def isok(self):
        ok = self.__bus.read_byte_data(self.__dev_id, _RA_WHO_AM_I) == 0x68
        return ok
    
    def isreadyFIFO(self, packet_size):
        FIFO_count = self.get_FIFO_count()
        mpu_int_status = self.get_int_status()

        # If overflow is detected by status or fifo count we want to reset
        if (FIFO_count == 1024) or (mpu_int_status & 0x10):
            self.reset_FIFO()
            print('overflow!')

            return 0
        # Check if fifo data is ready
        elif (mpu_int_status & 0x02):
            # Wait until packet_size number of bytes are ready for reading, default
            # is 42 bytes
            while FIFO_count < packet_size: # 等待直到 FIFO_count > 42
                FIFO_count = self.get_FIFO_count()

            return 1
        
        return 0

    # Core bit and byte operations
    def read_bit(self, a_reg_add, a_bit_position):
        return self.read_bits(a_reg_add, a_bit_position, 1)

    def write_bit(self, a_reg_add, a_bit_num, a_bit):
        byte = self.__bus.read_byte_data(self.__dev_id, a_reg_add)
        if a_bit:
            byte |= 1 << a_bit_num # 高num位1
        else:
            byte &= ~(1 << a_bit_num) # 高num位0
        self.__bus.write_byte_data(
            self.__dev_id, a_reg_add, byte)

    def read_bits(self, a_reg_add, a_bit_start, a_length):
        byte = self.__bus.read_byte_data(self.__dev_id, a_reg_add)
        mask = ((1 << a_length) - 1) << (a_bit_start - a_length + 1)
        byte &= mask
        byte >>= a_bit_start - a_length + 1
        return byte

    def write_bits(self, a_reg_add, a_bit_start, a_length, a_data):
        byte = self.__bus.read_byte_data(self.__dev_id, a_reg_add)
        mask = ((1 << a_length) - 1) << (a_bit_start - a_length + 1)
        # Get data in position and zero all non-important bits in data
        a_data <<= a_bit_start - a_length + 1
        a_data &= mask
        # Clear all important bits in read byte and combine with data
        byte &= ~mask
        byte = byte | a_data
        # Write the data to the I2C device
        self.__bus.write_byte_data(
            self.__dev_id, a_reg_add, byte  )

    def read_memory_byte(self):
        return self.__bus.read_byte_data(self.__dev_id, _RA_MEM_R_W)

    def read_bytes(self, a_data_list, a_address, a_length):
        if a_length > len(a_data_list):
            print('read_bytes, length of passed list too short')
            return a_data_list

        for x in range(0, a_length):
            a_data_list[x] = self.__bus.read_byte_data(self.__dev_id,
                                                       a_address + x)
        return a_data_list

    def write_memory_block(self, a_data_list, a_data_size, a_bank, a_address,
                           a_verify):
        success = True
        self.set_memory_bank(a_bank)
        self.set_memory_start_address(a_address)

        # For each a_data_item we want to write it to the board to a certain
        # memory bank and address
        for i in range(0, a_data_size):
            # Write each data to memory
            self.__bus.write_byte_data(self.__dev_id, _RA_MEM_R_W,
                                       a_data_list[i])

            if a_verify:
                self.set_memory_bank(a_bank)
                self.set_memory_start_address(a_address)
                verify_data = self.__bus.read_byte_data(self.__dev_id,
                                                        _RA_MEM_R_W)
                if verify_data != a_data_list[i]:
                    success = False

            # If we've filled the bank, change the memory bank
            if a_address == 255:
                a_address = 0
                a_bank += 1
                self.set_memory_bank(a_bank)
            else:
                a_address += 1

            # Either way update the memory address
            self.set_memory_start_address(a_address)

        return success

    def wake_up(self):
        self.write_bit(
            _RA_PWR_MGMT_1, _PWR1_SLEEP_BIT, 0)

    def set_clock_source(self, a_source):
        self.write_bits(_RA_PWR_MGMT_1, _PWR1_CLKSEL_BIT,
                        _PWR1_CLKSEL_LENGTH, a_source)

    def set_full_scale_gyro_range(self, a_data):
        self.write_bits(_RA_GYRO_CONFIG,
                        _GCONFIG_FS_SEL_BIT,
                        _GCONFIG_FS_SEL_LENGTH, a_data)

    def set_full_scale_accel_range(self, a_data):
        self.write_bits(_RA_ACCEL_CONFIG,
                        _ACONFIG_AFS_SEL_BIT,
                        _ACONFIG_AFS_SEL_LENGTH, a_data)

    def reset(self):
        self.write_bit(_RA_PWR_MGMT_1,
                       _PWR1_DEVICE_RESET_BIT, 1)

    def set_sleep_enabled(self, a_enabled):
        set_bit = 0
        if a_enabled:
            set_bit = 1
        self.write_bit(_RA_PWR_MGMT_1,
                       _PWR1_SLEEP_BIT, set_bit)

    def set_memory_bank(self, a_bank, a_prefetch_enabled=False,
                        a_user_bank=False):
        a_bank &= 0x1F
        if a_user_bank:
            a_bank |= 0x20
        if a_prefetch_enabled:
            a_bank |= 0x20
        self.__bus.write_byte_data(
            self.__dev_id, _RA_BANK_SEL, a_bank)

    def set_memory_start_address(self, a_address):
        self.__bus.write_byte_data(
            self.__dev_id, _RA_MEM_START_ADDR, a_address)

    def get_x_gyro_offset_TC(self):
        return self.read_bits(_RA_XG_OFFS_TC,
                              _TC_OFFSET_BIT,
                              _TC_OFFSET_LENGTH)

    def set_x_gyro_offset_TC(self, a_offset):
        self.write_bits(_RA_XG_OFFS_TC,
                        _TC_OFFSET_BIT,
                        _TC_OFFSET_LENGTH, a_offset)

    def get_y_gyro_offset_TC(self):
        return self.read_bits(_RA_YG_OFFS_TC,
                              _TC_OFFSET_BIT,
                              _TC_OFFSET_LENGTH)

    def set_y_gyro_offset_TC(self, a_offset):
        self.write_bits(_RA_YG_OFFS_TC,
                        _TC_OFFSET_BIT,
                        _TC_OFFSET_LENGTH, a_offset)

    def get_z_gyro_offset_TC(self):
        return self.read_bits(_RA_ZG_OFFS_TC,
                              _TC_OFFSET_BIT,
                              _TC_OFFSET_LENGTH)

    def set_z_gyro_offset_TC(self, a_offset):
        self.write_bits(_RA_ZG_OFFS_TC,
                        _TC_OFFSET_BIT,
                        _TC_OFFSET_LENGTH, a_offset)

    def set_slave_address(self, a_num, a_address):
        self.__bus.write_byte_data(
            self.__dev_id, _RA_I2C_SLV0_ADDR + a_num * 3, a_address)

    def set_I2C_master_mode_enabled(self, a_enabled):
        bit = 0
        if a_enabled:
            bit = 1
        self.write_bit(_RA_USER_CTRL,
                       _USERCTRL_I2C_MST_EN_BIT, bit)

    def reset_I2C_master(self):
        self.write_bit(_RA_USER_CTRL,
                       _USERCTRL_I2C_MST_RESET_BIT, 1)

    def write_prog_memory_block(self, a_data_list_index, a_data_size, a_bank=0,
                                a_address=0, a_verify=True):
        ''' 最快速度 '''
        with open( 'dmp.bin' , 'rb') as f:
            f.seek( a_data_list_index )
            a_data_list = f.read(a_data_size) # 读取一个数
        return self.write_memory_block(a_data_list, a_data_size, a_bank,
                                       a_address, a_verify)

    def write_DMP_configuration_set(self, a_data_list, a_data_size):
        index = 0
        while index < a_data_size:
            bank = a_data_list[index]
            offset = a_data_list[index + 1]
            length = a_data_list[index + 2]
            index += 3
            success = False

            # Normal case
            if length > 0:
                data_selection = list()
                for subindex in range(0, length):
                    data_selection.append(a_data_list[index + subindex])
                success = self.write_memory_block(data_selection, length, bank,
                                                  offset, True)
                index += length
            # Special undocumented case
            else:
                special = a_data_list[index]
                index += 1
                if special == 0x01:
                    # TODO Figure out if write8 can return True/False
                    success = self.__bus.write_byte_data(
                        self.__dev_id, _RA_INT_ENABLE, 0x32)

            if success == False:
                # TODO implement error messagemajigger
                return False
                pass
        return True

    def write_prog_dmp_configuration(self, a_data_list_index, freq_divider, a_data_size):
        with open( 'dmp.bin' , 'rb') as f:
            f.seek( a_data_list_index )
            a_data_list = f.read(a_data_size) 
            a_data_list = bytearray(a_data_list)
            a_data_list[-1] = freq_divider
        return self.write_DMP_configuration_set(a_data_list, a_data_size)

    def set_int_enable(self, a_enabled):
        self.__bus.write_byte_data(
            self.__dev_id, _RA_INT_ENABLE, a_enabled)

    def set_rate(self, a_rate):
        self.__bus.write_byte_data(
            self.__dev_id, _RA_SMPLRT_DIV, a_rate)

    def set_external_frame_sync(self, a_sync):
        self.write_bits(_RA_CONFIG,
                        _CFG_EXT_SYNC_SET_BIT,
                        _CFG_EXT_SYNC_SET_LENGTH, a_sync)

    def set_DLF_mode(self, a_mode):
        self.write_bits(_RA_CONFIG, _CFG_DLPF_CFG_BIT,
                        _CFG_DLPF_CFG_LENGTH, a_mode)

    def get_DMP_config_1(self):
        return self.__bus.read_byte_data(self.__dev_id, _RA_DMP_CFG_1)

    def set_DMP_config_1(self, a_config):
        self.__bus.write_byte_data(
            self.__dev_id, _RA_DMP_CFG_1, a_config)

    def get_DMP_config_2(self):
        return self.__bus.read_byte_data(self.__dev_id, _RA_DMP_CFG_2)

    def set_DMP_config_2(self, a_config):
        self.__bus.write_byte_data(
            self.__dev_id, _RA_DMP_CFG_2, a_config)

    def set_OTP_bank_valid(self, a_enabled):
        bit = 0
        if a_enabled:
            bit = 1
        self.write_bit(_RA_XG_OFFS_TC,
                       _TC_OTP_BNK_VLD_BIT, bit)

    def get_OTP_bank_valid(self):
        return self.read_bit(_RA_XG_OFFS_TC,
                             _TC_OTP_BNK_VLD_BIT)

    def set_motion_detection_threshold(self, a_threshold):
        self.__bus.write_byte_data(
            self.__dev_id, _RA_MOT_THR, a_threshold)

    def set_zero_motion_detection_threshold(self, a_threshold):
        self.__bus.write_byte_data(
            self.__dev_id, _RA_ZRMOT_THR, a_threshold)

    def set_motion_detection_duration(self, a_duration):
        self.__bus.write_byte_data(
            self.__dev_id, _RA_MOT_DUR, a_duration)

    def set_zero_motion_detection_duration(self, a_duration):
        self.__bus.write_byte_data(
            self.__dev_id, _RA_ZRMOT_DUR, a_duration)

    def set_FIFO_enabled(self, a_enabled):
        bit = 0
        if a_enabled:
            bit = 1
        self.write_bit(_RA_USER_CTRL,
                       _USERCTRL_FIFO_EN_BIT, bit)

    def set_DMP_enabled(self, a_enabled):
        bit = 0
        if a_enabled:
            bit = 1
        self.write_bit(_RA_USER_CTRL,
                       _USERCTRL_DMP_EN_BIT, bit)

    def reset_DMP(self):
        self.write_bit(_RA_USER_CTRL,
                       _USERCTRL_DMP_RESET_BIT, True)

    def dmp_initialize(self):
        # Reset the MPU
        self.reset()
        # time.Sleep a bit while resetting
        time.sleep(50 / 1000)
        # Disable time.sleep mode
        self.set_sleep_enabled(0)

        # get MPU hardware revision
        if self.__debug:
            print('Selecting user bank 16')
        self.set_memory_bank(0x10, True, True)

        if self.__debug:
            print('Selecting memory byte 6')
        self.set_memory_start_address(0x6)

        if self.__debug:
            print('Checking hardware revision')
        HW_revision = self.read_memory_byte()
        if self.__debug:
            print('Revision @ user[16][6] = ' + hex(HW_revision))

        if self.__debug:
            print('Resetting memory bank selection to 0')
        self.set_memory_bank(0)

        # check OTP bank valid
        # TODO Find out what OTP is
        OTP_valid = self.get_OTP_bank_valid()
        if self.__debug:
            if OTP_valid:
                print('OTP bank is valid')
            else:
                print('OTP bank is invalid')

        # get X/Y/Z gyro offsets
        if self.__debug:
            print('Reading gyro offet TC values')
        x_g_offset_TC = self.get_x_gyro_offset_TC()
        y_g_offset_TC = self.get_y_gyro_offset_TC()
        z_g_offset_TC = self.get_z_gyro_offset_TC()
        if self.__debug:
            print("X gyro offset = ", repr(x_g_offset_TC))
            print("Y gyro offset = ", repr(y_g_offset_TC))
            print("Z gyro offset = ", repr(z_g_offset_TC))

        # setup weird slave stuff (?)
        if self.__debug:
            print('Setting slave 0 address to 0x7F')
        self.set_slave_address(0, 0x7F)
        if self.__debug:
            print('Disabling I2C Master mode')
        self.set_I2C_master_mode_enabled(False)
        if self.__debug:
            print('Setting slave 0 address to 0x68 (self)')
        self.set_slave_address(0, 0x68)
        if self.__debug:
            print('Resetting I2C Master control')
        self.reset_I2C_master()
        # Wait a bit for the device to register the changes
        time.sleep(20 / 1000)

        # load DMP code into memory banks
        if self.__debug:
            print('Writing DMP code to MPU memory banks ' +
                  repr(_DMP_CODE_SIZE) + ' bytes')
        if self.write_prog_memory_block(_dmp_Memory, _DMP_CODE_SIZE):
            # TODO Check if we've actually verified this
            if self.__debug:
                print('Success! DMP code written and verified')

            # Write DMP configuration
            if self.__debug:
                print('Writing DMP configuration to MPU memory banks ' +
                      repr(_DMP_CONFIG_SIZE) + ' bytes in config')
                
            if self.write_prog_dmp_configuration(_dmp_Config,self.freq_divider,
                                                 _DMP_CONFIG_SIZE):
                if self.__debug:
                    print('Success! DMP configuration written and verified.')
                    print('Setting clock source to Z gyro')
                self.set_clock_source(_CLOCK_PLL_ZGYRO)

                if self.__debug:
                    print('Setting DMP and FIFO_OFLOW interrupts enabled')
                self.set_int_enable(0x12)

                if self.__debug:
                    print('Setting sample rate to 200Hz')
                self.set_rate(4)

                if self.__debug:
                    print('Setting external frame sync to TEMP_OUT_L[0]')
                self.set_external_frame_sync(_EXT_SYNC_TEMP_OUT_L)

                if self.__debug:
                    print('Setting DLPF bandwidth to 42Hz')
                self.set_DLF_mode(_DLPF_BW_42)

                if self.__debug:
                    print('Setting gyro sensitivity to +/- 2000 deg/sec')
                self.set_full_scale_gyro_range(_GYRO_FS_2000)

                if self.__debug:
                    print('Setting DMP configuration bytes (function unknown)')
                self.set_DMP_config_1(0x03)
                self.set_DMP_config_2(0x00)

                if self.__debug:
                    print('Clearing OTP Bank flag')
                self.set_OTP_bank_valid(False)

                if self.__debug:
                    print('Setting X/Y/Z gyro offset TCs to previous values')
                self.set_x_gyro_offset_TC(x_g_offset_TC)
                self.set_y_gyro_offset_TC(y_g_offset_TC)
                self.set_z_gyro_offset_TC(z_g_offset_TC)

                # Uncomment this to zero offsets when dmp_initialize is called
                # if self.__debug:
                #    print('Setting X/Y/Z gyro user offsets to zero')
                # self.set_x_gyro_offset(0)
                # self.set_y_gyro_offset(0)
                # self.set_z_gyro_offset(0)

                if self.__debug:
                    print('Writing final memory update 1/7 (function unknown)')
                pos = 0
                j = 0
                with open( 'dmp.bin' , 'rb') as f:
                    f.seek( _dmp_Updates )
                    dmpUpdates = f.read(_DMP_UPDATES_SIZE) 
                dmp_update = [0] * 16
                while (j < 4) or (j < dmp_update[2] + 3):
                    dmp_update[j] = dmpUpdates[pos]
                    pos += 1
                    j += 1
                # Write as block from pos 3
                self.write_memory_block(dmp_update[3:], dmp_update[2],
                                        dmp_update[0], dmp_update[1], True)

                if self.__debug:
                    print('Writing final memory update 2/7 (function unknown)')
                j = 0
                while (j < 4) or (j < dmp_update[2] + 3):
                    dmp_update[j] = dmpUpdates[pos]
                    pos += 1
                    j += 1
                # Write as block from pos 3
                self.write_memory_block(dmp_update[3:], dmp_update[2],
                                        dmp_update[0], dmp_update[1], True)

                if self.__debug:
                    print('Resetting FIFO')
                self.reset_FIFO()

                if self.__debug:
                    print('Reading FIFO count')
                FIFO_count = self.get_FIFO_count()

                if self.__debug:
                    print('FIFO count: ' + repr(FIFO_count))

                if self.__debug:
                    print('Getting FIFO buffer')
                FIFO_buffer = [0] * 128
                FIFO_buffer = self.get_FIFO_bytes(FIFO_count)

                if self.__debug:
                    print('Setting motion detection threshold to 2')
                self.set_motion_detection_threshold(2)

                if self.__debug:
                    print('Setting zero-motion detection threshold to 156')
                self.set_zero_motion_detection_threshold(156)

                if self.__debug:
                    print('Setting motion detection duration to 80')
                self.set_motion_detection_duration(80)

                if self.__debug:
                    print('Setting zero-motion detection duration to 0')
                self.set_zero_motion_detection_duration(0)

                if self.__debug:
                    print('Resetting FIFO')
                self.reset_FIFO()

                if self.__debug:
                    print('Enabling FIFO')
                self.set_FIFO_enabled(True)

                if self.__debug:
                    print('Enabling DMP')
                self.set_DMP_enabled(True)

                if self.__debug:
                    print('Resetting DMP')
                self.reset_DMP()

                if self.__debug:
                    print('Writing final memory update 3/7 (function unknown)')
                j = 0
                while (j < 4) or (j < dmp_update[2] + 3):
                    dmp_update[j] = dmpUpdates[pos]
                    pos += 1
                    j += 1
                # Write as block from pos 3
                self.write_memory_block(dmp_update[3:], dmp_update[2],
                                        dmp_update[0], dmp_update[1], True)

                if self.__debug:
                    print('Writing final memory update 4/7 (function unknown)')
                j = 0
                while (j < 4) or (j < dmp_update[2] + 3):
                    dmp_update[j] = dmpUpdates[pos]
                    pos += 1
                    j += 1
                # Write as block from pos 3
                self.write_memory_block(dmp_update[3:], dmp_update[2],
                                        dmp_update[0], dmp_update[1], True)

                if self.__debug:
                    print('Writing final memory update 5/7 (function unknown)')
                j = 0
                while (j < 4) or (j < dmp_update[2] + 3):
                    dmp_update[j] = dmpUpdates[pos]
                    pos += 1
                    j += 1
                # Write as block from pos 3
                self.write_memory_block(dmp_update[3:], dmp_update[2],
                                        dmp_update[0], dmp_update[1], True)

                if self.__debug:
                    print('Waiting for FIFO count > 2')
                FIFO_count = self.get_FIFO_count()
                while FIFO_count < 3:
                    FIFO_count = self.get_FIFO_count()

                if self.__debug:
                    print('Current FIFO count = ' + repr(FIFO_count))
                    print('Reading FIFO data')
                FIFO_buffer = self.get_FIFO_bytes(FIFO_count)

                if self.__debug:
                    print('Reading interrupt status')
                MPU_int_status = self.get_int_status()

                if self.__debug:
                    print('Current interrupt status = ' + hex(MPU_int_status))
                    print('Writing final memory update 6/7 (function unknown)')
                j = 0
                while (j < 4) or (j < dmp_update[2] + 3):
                    dmp_update[j] = dmpUpdates[pos]
                    pos += 1
                    j += 1
                # Write as block from pos 3
                self.write_memory_block(dmp_update[3:], dmp_update[2],
                                        dmp_update[0], dmp_update[1], True)

                if self.__debug:
                    print('Waiting for FIFO count > 2')
                FIFO_count = self.get_FIFO_count()
                while FIFO_count < 3:
                    FIFO_count = self.get_FIFO_count()

                if self.__debug:
                    print('Current FIFO count = ' + repr(FIFO_count))
                    print('Reading FIFO count')
                FIFO_buffer = self.get_FIFO_bytes(FIFO_count)

                if self.__debug:
                    print('Reading interrupt status')
                MPU_int_status = self.get_int_status()

                if self.__debug:
                    print('Current interrupt status = ' + hex(MPU_int_status))
                    print('Writing final memory update 7/7 (function unknown)')
                j = 0
                while (j < 4) or (j < dmp_update[2] + 3):
                    dmp_update[j] = dmpUpdates[pos]
                    pos += 1
                    j += 1
                # Write as block from pos 3
                self.write_memory_block(dmp_update[3:], dmp_update[2],
                                        dmp_update[0], dmp_update[1], True)

                if self.__debug:
                    print('DMP is good to go! Finally.')
                    print('Disabling DMP (you turn it on later)')
                self.set_DMP_enabled(False)

                if self.__debug:
                    print('Setting up internal 42 byte DMP packet buffer')
                self.__DMP_packet_size = 42

                if self.__debug:
                    print(
                        'Resetting FIFO and clearing INT status one last time')
                self.reset_FIFO()
                self.get_int_status()

            else:
                if self.__debug:
                    print('Configuration block loading failed')
                return 2

        else:
            if self.__debug:
                print('Main binary block loading failed')
            return 1

        if self.__debug:
            print('DMP initialization was successful')
        return 0

    # Acceleration and gyro offset setters and getters
    def set_x_accel_offset(self, a_offset):
        self.__bus.write_byte_data(self.__dev_id, _RA_XA_OFFS_H,
                                   a_offset >> 8)
        self.__bus.write_byte_data(self.__dev_id, _RA_XA_OFFS_L_TC,
                                   a_offset & 0xFF )

    def set_y_accel_offset(self, a_offset):
        self.__bus.write_byte_data(self.__dev_id, _RA_YA_OFFS_H,
                                   a_offset >> 8)
        self.__bus.write_byte_data(self.__dev_id, _RA_YA_OFFS_L_TC,
                                   a_offset & 0xFF )

    def set_z_accel_offset(self, a_offset):
        self.__bus.write_byte_data(self.__dev_id, _RA_ZA_OFFS_H,
                                   a_offset >> 8)
        self.__bus.write_byte_data(self.__dev_id, _RA_ZA_OFFS_L_TC,
                                   a_offset & 0xFF )

    def set_x_gyro_offset(self, a_offset):
        self.__bus.write_byte_data(self.__dev_id, _RA_XG_OFFS_USRH,
                                   a_offset >> 8)
        self.__bus.write_byte_data(self.__dev_id, _RA_XG_OFFS_USRL,
                                   a_offset & 0xFF )

    def set_y_gyro_offset(self, a_offset):
        self.__bus.write_byte_data(self.__dev_id, _RA_YG_OFFS_USRH,
                                   a_offset >> 8)
        self.__bus.write_byte_data(self.__dev_id, _RA_YG_OFFS_USRL,
                                   a_offset & 0xFF )

    def set_z_gyro_offset(self, a_offset):
        self.__bus.write_byte_data(self.__dev_id, _RA_ZG_OFFS_USRH,
                                   a_offset >> 8)
        self.__bus.write_byte_data(self.__dev_id, _RA_ZG_OFFS_USRL,
                                   a_offset & 0xFF )

    # Main interfacing functions to get raw data from MPU
    @property
    def acceleration(self):
        raw_data = self.__bus.read_i2c_block_data(self.__dev_id,
                                                  _RA_ACCEL_XOUT_H, 6)
        x,y,z = struct.unpack(">hhh", raw_data)
        return x,y,z
    
    @property
    def rotation(self):
        raw_data = self.__bus.read_i2c_block_data(self.__dev_id,
                                                  _RA_GYRO_XOUT_H, 6)
        x,y,z = struct.unpack(">hhh", raw_data)
        return x,y,z

    # Interfacing functions to get data from FIFO buffe
    def DMP_get_FIFO_packet_size(self):
        return self.__DMP_packet_size

    def reset_FIFO(self):
        self.write_bit(_RA_USER_CTRL,
                       _USERCTRL_FIFO_RESET_BIT, True)

    def get_FIFO_count(self):
        # data = [0] * 2
        # data = self.read_bytes(data, _RA_FIFO_COUNTH, 2)
        data = self.__bus.read_i2c_block_data(self.__dev_id,_RA_FIFO_COUNTH, 2)
        count, = struct.unpack(">h",data)
        return count

    def get_FIFO_bytes(self, a_FIFO_count):
        # return_list = list()
        return_list = bytearray()
        for index in range(0, a_FIFO_count):
            return_list.append( self.__bus.read_byte_data(self.__dev_id, _RA_FIFO_R_W))
        return return_list

    def get_int_status(self):
        return self.__bus.read_byte_data(self.__dev_id,
                                         _RA_INT_STATUS)

    # Data retrieval from received FIFO buffer
    def DMP_get_quaternion_int16(self, a_FIFO_buffer):
        w, = struct.unpack(">h",a_FIFO_buffer[0:2])
        x, = struct.unpack(">h",a_FIFO_buffer[4:6])
        y, = struct.unpack(">h",a_FIFO_buffer[8:10])
        z, = struct.unpack(">h",a_FIFO_buffer[12:14])
        return w, x, y, z

    def DMP_get_quaternion(self, a_FIFO_buffer):
        w,x,y,z= self.DMP_get_quaternion_int16(a_FIFO_buffer)
        w = w / 16384.0
        x = x / 16384.0
        y = y / 16384.0
        z = z / 16384.0
        return w, x, y, z

    def DMP_get_acceleration_int16(self, a_FIFO_buffer):
        x, = struct.unpack(">h", a_FIFO_buffer[28:30])
        y, = struct.unpack(">h", a_FIFO_buffer[32:34])
        z, = struct.unpack(">h", a_FIFO_buffer[36:38])
        return x, y, z

    def DMP_get_gravity(self, w,x,y,z):
        x = 2.0 * ( x *  z -  w *  y)
        y = 2.0 * ( w *  x +  y *  z)
        z = 1.0 * ( w *  w -  x *  x -
                    y *  y +  z *  z)
        return x, y, z

    def DMP_get_euler(self, w,x,y,z):
        psi = math.atan2(2* x* y - 2* w* z,
                         2* w* w + 2* x* x - 1)
        theta = -math.asin(2* x* z + 2* w* y)
        phi = math.atan2(2* y* z - 2* w* x,
                         2* w* w + 2* z* z - 1)
        return psi, theta, phi

    def DMP_get_roll_pitch_yaw(self, w,x,y,z):
        norm = math.sqrt( w**2 +  x**2 +  y**2 +  z**2)
        w =  w / norm
        x =  x / norm
        y =  y / norm
        z =  z / norm
        yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
        pitch = math.asin(2 * (w * y - z * x))
        roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
        return roll, pitch, yaw

    def DMP_get_euler_roll_pitch_yaw(self, w,x,y,z):
        roll, pitch, yaw = self.DMP_get_roll_pitch_yaw(w,x,y,z)
        roll = roll * (180.0/math.pi)
        pitch = pitch * (180.0/math.pi)
        yaw = yaw * (180.0/math.pi)
        return roll, pitch, yaw

    def DMP_get_linear_accel(self, a_vector_raw, a_vect_grav):
        x = a_vector_raw.x - a_vect_grav.x*8192
        y = a_vector_raw.y - a_vect_grav.y*8192
        z = a_vector_raw.z - a_vect_grav.z*8192
        return x, y, z



import numpy as np
import imu
import asyncio

# accelerometer/gyroscope register addresses
# from register map - https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Register-Map1.pdf
# https://www.st.com/resource/en/datasheet/lsm6dsox.pdf

DEVICE_ADDRESS = [0x6A, 0x1C]

#######################################

# Accelerometer - 208 Hz normal mode, ±2g, output from 1st stage digital filter
CTRL1_XL = 0x10
# Gyroscope - 208 Hz normal mode, 250 dps, 00, FS
CTRL2_G = 0x11
# disable i3c
CTRL9_XL = 0x18

# low in first position, high in second position
ACC_REGISTERS = [[0x28, 0x29], [0x2A, 0x2B], [0x2C, 0x2D]]
GYRO_REGISTERS = [[0x22, 0x23], [0x24, 0x25], [0x26, 0x27]]

ACC_BIAS = [[6.10987778e-05, -3.69227774e-03],
            [6.09998042e-05, -8.51929284e-03],
            [6.04545451e-05, -6.48953360e-03]]

GYRO_SCALE = 8.75 / 1000
GYRO_BIAS = [11.367, -31.793, -24.028]

##########################################

CTRL_REG1_M = 0x20
CTRL_REG2_M = 0x21
CTRL_REG3_M = 0x22
CTRL_REG4_M = 0x23

XY_OP_MODE_UHIGH = 0x60  # Ultra-High-Performance Mode,  ODR [Hz]: 155, FAST_ODR: 1
Z_OP_MODE_UHIGH = 0x0C
D_RATE_80 = 0x1C  # ODR = 80 Hz
MAG_GAIN_4G = 0x00  # Full scale = +/-4 Gauss, LSB first
MEAS_CONT = 0x00  # Continuous-Conversion Mode

MAG_REGISTERS = [[0x28, 0x29], [0x2A, 0x2B], [0x2C, 0x2D]]

MAG_BIAS = [[1.051173994115047, 0.023163617823031002, 460.15600170384727],
            [0.02316361782303098, 1.0104848800632054, -1771.4260099481783],
            [0.0, 0.0, 1.0]]

acc_setup = [[CTRL9_XL, 1],
             [CTRL1_XL, 0b01010000],
             [CTRL2_G, 0b01010000]]

mag_setup = [[CTRL_REG1_M, XY_OP_MODE_UHIGH],
             [CTRL_REG1_M, D_RATE_80],
             [CTRL_REG2_M, MAG_GAIN_4G],
             [CTRL_REG3_M, MEAS_CONT],
             [CTRL_REG4_M, Z_OP_MODE_UHIGH]]

# prepend device address to each register and join arrays
setup = np.column_stack((np.full((len(acc_setup), 1), DEVICE_ADDRESS[0]), acc_setup))
setup = np.concatenate((setup, np.column_stack((np.full((len(mag_setup), 1), DEVICE_ADDRESS[1]), mag_setup))), axis=0)

elevation = imu.IMU(DEVICE_ADDRESS, GYRO_SCALE, ACC_REGISTERS, GYRO_REGISTERS, MAG_REGISTERS, ACC_BIAS, GYRO_BIAS,
                    MAG_BIAS, *setup)

asyncio.run(elevation.stream_readings([1, 2, [3]]))

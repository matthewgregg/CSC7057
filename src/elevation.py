import numpy as np

import imu

# accelerometer/gyroscope register addresses
# from register map - https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Register-Map1.pdf
# https://www.st.com/resource/en/datasheet/lsm6dsox.pdf

DEVICE_ADDRESS = [0x6A, 0x1C]

#######################################

# 208 Hz normal mode, ±2g, output from 1st stage digital filter
CTRL1_XL = 0x10
# 208 Hz normal mode, 250 dps, 00, FS
CTRL2_G = 0x11
# disable i3c
CTRL9_XL = 0x18

# low in first position, high in second position
ACC_REGISTERS = [[0x22, 0x23], [0x24, 0x25], [0x26, 0x27]]
GYRO_REGISTERS = [[0x28, 0x29], [0x2A, 0x2B], [0x2C, 0x2D]]

ACC_BIAS = [[-0.00285675, 0.00944442],
            [-0.00323874, -0.09694955],
            [-0.00226641, -0.05384152]]

GYRO_SCALE = 8.75 / 1000
GYRO_BIAS = [2558.33333333, -5482.33333333, -4151.33333333]

##########################################

CTRL_REG1_M = 0x20
CTRL_REG2_M = 0x21
CTRL_REG3_M = 0x22
CTRL_REG4_M = 0x23

XY_OP_MODE_UHIGH = 0x60  # Ultra-High-Performance Mode,  ODR [Hz]: 155, FAST_ODR: 1
Z_OP_MODE_UHIGH = 0x0C
D_RATE_0_685 = 0x00  # ODR = 0.625 Hz
MAG_GAIN_4G = 0x00  # Full scale = +/-4 Gauss, LSB first
MEAS_CONT = 0x00  # Continuous-Conversion Mode

MAG_REGISTERS = [[0x28, 0x29], [0x2A, 0x2B], [0x2C, 0x2D]]

MAG_BIAS = [[1.0076664788827325, 0.03085027311558635, 2894.471358004767],
            [0.030850273115586305, 1.1241429560902958, 100.64453832541044],
            [0.0, 0.0, 1.0]]

MAG_X_L = 0x28
MAG_X_H = 0x29
MAG_Y_L = 0x2A
MAG_Y_H = 0x2B
MAG_Z_L = 0x2C
MAG_Z_H = 0x2D

acc_setup = [[CTRL9_XL, 1],
             [CTRL1_XL, 0b01010000],
             [CTRL2_G, 0b01010000]]

mag_setup = [[CTRL_REG1_M, XY_OP_MODE_UHIGH],
             [CTRL_REG4_M, Z_OP_MODE_UHIGH],
             [CTRL_REG1_M, D_RATE_0_685],
             [CTRL_REG2_M, MAG_GAIN_4G],
             [CTRL_REG3_M, MEAS_CONT]]

# prepend device address to each register and join arrays
setup = np.column_stack((np.full((len(acc_setup), 1), DEVICE_ADDRESS[0]), acc_setup))
setup = np.concatenate((setup, np.column_stack((np.full((len(mag_setup), 1), DEVICE_ADDRESS[1]), mag_setup))), axis=0)

elevation = imu.IMU(DEVICE_ADDRESS, GYRO_SCALE, ACC_REGISTERS, GYRO_REGISTERS, MAG_REGISTERS, ACC_BIAS, GYRO_BIAS, MAG_BIAS, *setup)

import imu

# from register map - https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Register-Map1.pdf
# https://www.st.com/resource/en/datasheet/lsm6dsox.pdf
# ACC_GYRO_ADDRESS = 0x6A
# # 208 Hz normal mode, ±2g, output from 1st stage digital filter
# CTRL1_XL = 0x10
# # 208 Hz normal mode, 250 dps, 00, FS
# CTRL2_G = 0x11
#
# # high in first position, low in second position
# ACC_REGISTERS = [[0x23, 0x22], [0x25, 0x24], [0x27, 0x26]]
# GYRO_REGISTERS = [[0x29, 0x28], [0x2B, 0x2A], [0x2D, 0x2C]]
#
# ACC_BIAS = [[6.07401519e-05, -7.99851022e-02],
#             [6.08876489e-05, -7.61055061e-03],
#             [5.99174528e-05, 1.80092610e-01]]
# GYRO_SCALE = 8.75 / 1000
# GYRO_BIAS = [-8099.33333333, -8212.66666667, -5927.33333333]


ACC_GYRO_ADDRESS = 0x68
PWR_MGMT_1 = 0x6B
SMPLRT_DIV = 0x19
CONFIG = 0x1A
GYRO_CONFIG = 0x1B
INT_ENABLE = 0x38

# high in first position, low in second position
ACC_REGISTERS = [[0x23, 0x22], [0x25, 0x24], [0x27, 0x26]]
GYRO_REGISTERS = [[0x29, 0x28], [0x2B, 0x2A], [0x2D, 0x2C]]

ACC_BIAS = [[6.07401519e-05, -7.99851022e-02],
            [6.08876489e-05, -7.61055061e-03],
            [5.99174528e-05, 1.80092610e-01]]
GYRO_SCALE = 1 / 16.4  # 131?
GYRO_BIAS = [-8099.33333333, -8212.66666667, -5927.33333333]

# SMPLRT_DIV - Sample Rate = Gyroscope Output Rate / (1 + SMPLRT_DIV)
# gyro output rate is 1 kHz since DLPF is enabled, making the sample rate 125 Hz
# PWR_MGMT_1 - PLL with X axis gyroscope reference for improved stability
# CONFIG - low pass filter to max value (6) - 5 Hz with 19 ms delay for acc, 18.6 ms delay for gyro
# GYRO_CONFIG - set gyro to ± 2000º/s (3)
# INT_ENABLE - enable Data Ready interrupt, occurs when write op to all of the sensor registers is completed

setup = [[SMPLRT_DIV, 7], [PWR_MGMT_1, 1], [CONFIG, int('0000110', 2)], [GYRO_CONFIG, 24], [INT_ENABLE, 1]]

# setup = [[CTRL1_XL, 0b01010000], [CTRL2_G, 0b01010000]]

elevation = imu.IMU(ACC_GYRO_ADDRESS, GYRO_SCALE, ACC_REGISTERS, GYRO_REGISTERS, None, None, ACC_BIAS, GYRO_BIAS, *setup)

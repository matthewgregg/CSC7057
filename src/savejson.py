import json

DEVICE_ADDRESS = [0x6A, 0x1C]

#######################################

# Accelerometer - 208 Hz normal mode, ±2g, output from 1st stage digital filter
CTRL1_XL = 0x10
# Gyroscope - 208 Hz normal mode, 250 dps, 00, FS
CTRL2_G = 0x11
# disable i3c
CTRL9_XL = 0x18

# lsb/low in first position, msb/high in second position (sensors are little endian, switch for big endian)
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

MAG_BIAS_A_INV = [[3.44057751e-04, 1.99840401e-05, 1.55847712e-07],
                  [1.99840401e-05, 3.62689168e-04, 9.02826449e-06],
                  [1.55847712e-07, 9.02826449e-06, 3.33093676e-04]]

MAG_BIAS_B = [[-757.24783875],
              [2323.69419632],
              [-151.7957312]]

MAG_BIAS = (MAG_BIAS_A_INV, MAG_BIAS_B)

acc_setup = [[CTRL9_XL, 1],
             [CTRL1_XL, 0b01010000],
             [CTRL2_G, 0b01010000]]

mag_setup = [[CTRL_REG1_M, XY_OP_MODE_UHIGH],
             [CTRL_REG1_M, D_RATE_80],
             [CTRL_REG2_M, MAG_GAIN_4G],
             [CTRL_REG3_M, MEAS_CONT],
             [CTRL_REG4_M, Z_OP_MODE_UHIGH]]

vals = {'accelerometer':
            {0x6A :
                 {'setup' : [[0x18, 1], [0x10, 0b01010000], [0x11, 0b01010000]],
                'accaddress' : [[0x28, 0x29], [0x2A, 0x2B], [0x2C, 0x2D]],
                'gyroaddress' : [[0x22, 0x23], [0x24, 0x25], [0x26, 0x27]],
                'gyroscale' : 8.75 / 1000,
                'accbias' : [[6.10987778e-05, -3.69227774e-03], [6.09998042e-05, -8.51929284e-03], [6.04545451e-05, -6.48953360e-03]],
                'gyrobias' : [11.367, -31.793, -24.028]},
        'magnetometer' :
            {0x1C :
                 {'setup' : [[0x20, 0x60], [0x20, 0x1C], [0x21, 0x00], [0x22, 0x00], [0x23, 0x0C]],
                  'magaddress' : [[0x28, 0x29], [0x2A, 0x2B], [0x2C, 0x2D]],
                  'magbiasa' : [[3.44057751e-04, 1.99840401e-05, 1.55847712e-07], [1.99840401e-05, 3.62689168e-04, 9.02826449e-06],[1.55847712e-07, 9.02826449e-06, 3.33093676e-04]],
                  'magbiasb' : [[-757.24783875], [2323.69419632], [-151.7957312]]}}},
        'motor' :
            {'elevation' : [24, 25, 8, 7], 'azimuth' : [12, 16, 20, 21]}}

json.dump(vals, open('deviceconfig.json', 'w'))
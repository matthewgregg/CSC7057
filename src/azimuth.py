import imu

DEVICE_ADDRESS = 0x1C

# from datasheet - https://www.st.com/resource/en/datasheet/lis3mdl.pdf
# LIS3MDL Magnetometer register
CTRL_REG1_M = 0x20
CTRL_REG2_M = 0x21
CTRL_REG3_M = 0x22
CTRL_REG4_M = 0x23
MAG_X_L = 0x28
MAG_Y_L = 0x2A
MAG_Z_L = 0x2C

# Mag X/Y Axes Operating Mode
XY_OP_MODE_LOW = 0x00  # Low-power Mode,  ODR [Hz]: 1000, FAST_ODR: 1
XY_OP_MODE_MED = 0x20  # Medium-Performance Mode,  ODR [Hz]: 560, FAST_ODR: 1
XY_OP_MODE_HIGH = 0x40  # High-Performance Mode,  ODR [Hz]: 300, FAST_ODR: 1
XY_OP_MODE_UHIGH = 0x60  # Ultra-High-Performance Mode,  ODR [Hz]: 155, FAST_ODR: 1

# Mag Z Axis Operating Mode
Z_OP_MODE_LOW = 0x00  # Low-power Mode
Z_OP_MODE_MED = 0x04  # Medium-Performance Mode
Z_OP_MODE_HIGH = 0x08  # High-Performance Mode
Z_OP_MODE_UHIGH = 0x0C  # Ultra-High-Performance Mode

# Mag Datarate configuration
D_RATE_0_685 = 0x00  # ODR = 0.625 Hz
D_RATE_1_25 = 0x04  # ODR = 1.25 Hz
D_RATE_2_5 = 0x08  # ODR = 2.5 Hz
D_RATE_5 = 0x0C  # ODR = 5 Hz
D_RATE_10 = 0x10  # ODR = 10 Hz
D_RATE_20 = 0x14  # ODR = 20 Hz
D_RATE_40 = 0x18  # ODR = 40 Hz
D_RATE_80 = 0x1C  # ODR = 80 Hz

# Magnetic Field Full-scale selection
MAG_GAIN_4G = 0x00  # Full scale = +/-4 Gauss, LSB first
MAG_GAIN_8G = 0x20  # Full scale = +/-8 Gauss
MAG_GAIN_12G = 0x40  # Full scale = +/-12 Gauss
MAG_GAIN_16G = 0x60  # Full scale = +/-16 Gauss

# Mag Measurement Mode
MEAS_CONT = 0x00  # Continuous-Conversion Mode
MEAS_SING = 0x01  # Single-Conversion Mode
MEAS_POWER_DOWN = 0x03  # Power-Down Mode

mag_bias = [[1.0076664788827325, 0.03085027311558635, 2894.471358004767],
            [0.030850273115586305, 1.1241429560902958, 100.64453832541044],
            [0.0, 0.0, 1.0]]


def mag_init() -> None:
    BUS.write_byte_data(DEVICE_ADDRESS, CTRL_REG1_M, XY_OP_MODE_UHIGH)
    BUS.write_byte_data(DEVICE_ADDRESS, CTRL_REG1_M, D_RATE_0_685)
    BUS.write_byte_data(DEVICE_ADDRESS, CTRL_REG2_M, MAG_GAIN_4G)
    BUS.write_byte_data(DEVICE_ADDRESS, CTRL_REG3_M, MEAS_CONT)
    BUS.write_byte_data(DEVICE_ADDRESS, CTRL_REG4_M, Z_OP_MODE_UHIGH)

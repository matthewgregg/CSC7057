import math

import smbus

BUS = smbus.SMBus(1)
DEVICE_ADDRESS = 0x1C

# from datasheet - https://www.st.com/resource/en/datasheet/lis3mdl.pdf
# LIS3MDL Magnetometer register
CTRL_REG1_M = 0x20
CTRL_REG2_M = 0x21
CTRL_REG3_M = 0x22
CTRL_REG4_M = 0x23
MAG_X_L = 0x28
MAG_X_H = 0x29
MAG_Y_L = 0x2A
MAG_Y_H = 0x2B
MAG_Z_L = 0x2C
MAG_Z_H = 0x2D

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


def read_sensor_data(addr) -> float:
    """
    Write data to an address on the bus
    """
    high = BUS.read_byte_data(DEVICE_ADDRESS, addr[0])
    low = BUS.read_byte_data(DEVICE_ADDRESS, addr[1])
    value = high * 256 + low  # (high << 8 | low)
    if value > 2 ** 16 / 2:
        value = value - 2 ** 16
    return value


BUS.write_byte_data(DEVICE_ADDRESS, CTRL_REG1_M, XY_OP_MODE_UHIGH)
BUS.write_byte_data(DEVICE_ADDRESS, CTRL_REG1_M, D_RATE_80)
BUS.write_byte_data(DEVICE_ADDRESS, CTRL_REG2_M, MAG_GAIN_4G)
BUS.write_byte_data(DEVICE_ADDRESS, CTRL_REG3_M, MEAS_CONT)
BUS.write_byte_data(DEVICE_ADDRESS, CTRL_REG4_M, Z_OP_MODE_UHIGH)


def apply_calibration(uncal_x, uncal_y, uncal_z) -> tuple[float, float, float]:
    x_cal_max_min = [-7656, 1020]
    y_cal_max_min = [-4324, 4398]
    z_cal_max_min = [-4364, 4144]

    x_cal_max_min = [-4508, 2294]
    y_cal_max_min = [-1024, 5448]
    z_cal_max_min = [-11767, -5097]

    uncal_x = uncal_x - (x_cal_max_min[0] + x_cal_max_min[1]) / 2
    uncal_y = uncal_y - (y_cal_max_min[0] + y_cal_max_min[1]) / 2
    uncal_z = uncal_z - (z_cal_max_min[0] + z_cal_max_min[1]) / 2

    # cal = [[1.0206151090681648, 0.03838325349919703, -318.0519914842439],
    #        [0.03838325349919701, 1.0714657460366654, 106.74930601148539],
    #        [0.0, 0.0, 1.0]]

    cal = [[1.0278762756271655, 0.055201472560838794, -556.5738414085462],
           [0.05520147256083878, 1.1093116818631081, -288.91935703975946],
           [0.0, 0.0, 1.0]]

    cal_x = uncal_x * cal[0][0] + uncal_y * cal[0][1] + cal[0][2]
    cal_y = uncal_x * cal[1][0] + uncal_y * cal[1][1] + cal[1][2]
    cal_z = uncal_z * cal[2][0] + uncal_z * cal[2][1] + cal[2][2]

    return cal_x, cal_y, cal_z


x_offset = [1e6, -1e6]
y_offset = [1e6, -1e6]
z_offset = [1e6, -1e6]
while True:
    m_x = read_sensor_data([MAG_X_H, MAG_X_L])
    m_y = read_sensor_data([MAG_Y_H, MAG_Y_L])
    m_z = read_sensor_data([MAG_Z_H, MAG_Z_L])

    x, y, z = apply_calibration(m_x, m_y, m_z)

    b = math.degrees(math.atan2(y, x))
    if b < 0:
        b += 360.0

    print(b)

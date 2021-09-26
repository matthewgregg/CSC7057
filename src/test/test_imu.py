import asyncio
from unittest import IsolatedAsyncioTestCase
import numpy as np
import imu
from datetime import datetime as dt, timedelta, timezone
import shared


class TestIMU(IsolatedAsyncioTestCase):
    time = dt.now(timezone.utc) + timedelta(seconds=10)
    shared.Running.value = True

    def test_constructor(self):
        DEVICE_ADDRESS = [0x6A, 0x1C]
        CTRL1_XL = 0x10
        CTRL2_G = 0x11
        CTRL9_XL = 0x18
        ACC_REGISTERS = [[0x28, 0x29], [0x2A, 0x2B], [0x2C, 0x2D]]
        GYRO_REGISTERS = [[0x22, 0x23], [0x24, 0x25], [0x26, 0x27]]
        ACC_BIAS = [[6.10987778e-05, -3.69227774e-03],
                    [6.09998042e-05, -8.51929284e-03],
                    [6.04545451e-05, -6.48953360e-03]]
        GYRO_SCALE = 8.75 / 1000
        GYRO_BIAS = [11.367, -31.793, -24.028]
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

        setup1 = np.column_stack((np.full((len(acc_setup), 1), DEVICE_ADDRESS[0]), acc_setup))
        mag_setup = [[CTRL_REG1_M, XY_OP_MODE_UHIGH],
                     [CTRL_REG1_M, D_RATE_80],
                     [CTRL_REG2_M, MAG_GAIN_4G],
                     [CTRL_REG3_M, MEAS_CONT],
                     [CTRL_REG4_M, Z_OP_MODE_UHIGH]]

        setup = np.concatenate((setup1, np.column_stack((np.full((len(mag_setup), 1), DEVICE_ADDRESS[1]), mag_setup))),
                               axis=0)

        i = imu.IMU(DEVICE_ADDRESS, GYRO_SCALE, ACC_REGISTERS, GYRO_REGISTERS, MAG_REGISTERS, ACC_BIAS, GYRO_BIAS,
                  MAG_BIAS, *setup)
        self.assertIsInstance(i.get_bearing(), float)

        i2 = imu.IMU(DEVICE_ADDRESS[0], GYRO_SCALE, ACC_REGISTERS, GYRO_REGISTERS, None, None, None,
                  None, *setup1)
        self.assertIsInstance(i2.get_bearing(), float)

    async def test_start_tasks(self):
        await asyncio.gather(imu.imu.stream_readings([0, 0, [self.time]]))
        self.time += timedelta(seconds=1)
        await asyncio.gather(*[imu.imu.stream_readings([0, 0, [self.time]]), self.get_bearing()])
        self.time += timedelta(seconds=1)
        await asyncio.gather(*[imu.imu.stream_readings([0, 0, [self.time]]), self.get_adjusted_elevation()])
        self.time += timedelta(seconds=1)
        await asyncio.gather(*[imu.imu.stream_readings([0, 0, [self.time]]), self.get_readings()])

    async def get_bearing(self):
        b = imu.imu.get_bearing()
        self.assertGreaterEqual(b, 0)
        self.assertLessEqual(b, 360)


    async def get_adjusted_elevation(self):
        e = imu.imu.get_adjusted_elevation()
        self.assertGreaterEqual(e, -90)
        self.assertLessEqual(e, 90)

    async def get_readings(self):
        roll, pitch, yaw = imu.imu.get_readings()
        self.assertGreaterEqual(roll, -90)
        self.assertLessEqual(roll, 90)
        self.assertGreaterEqual(pitch, -180)
        self.assertLessEqual(pitch, 180)
        self.assertGreaterEqual(yaw, -180)
        self.assertLessEqual(yaw, 180)

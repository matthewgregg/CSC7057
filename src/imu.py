import asyncio
import time
from typing import Union, Any
import smbus
import math
import numpy as np
import geomag
from scipy.optimize import curve_fit
from scipy import linalg
# import gps
import kalman
import shared


class IMU(object):
    BUS = smbus.SMBus(1)
    x = 0
    y = 0
    z = 0

    def __init__(self, device_address, gyro_scale, accel_add, gyro_add, mag_add=None, accel_bias=None, gyro_bias=None,
                 mag_bias=None, *args: list[int, int, int]):
        """
        Initialise IMU
        :param device_address: the accelerometer and gyroscope bus address
        :param gyro_scale: the factor to scale gyroscope data by (from datasheet)
        :param accel_add: the accelerometer's addresses
        :param gyro_add: the gyroscope's addresses
        :param mag_add: the magnetometer's addresses
        :param accel_bias: the accelerometer's bias
        :param gyro_bias: the gyroscope's bias
        :param mag_bias: the magnetometer bias
        :param args: miscellaneous setup addresses with device address and value
        """
        if accel_bias is None:
            accel_bias = [[1, 0], [1, 0], [1, 0]]

        if gyro_bias is None:
            gyro_bias = [0, 0, 0]

        if mag_bias is None:
            mag_bias = [[1, 0, 0], [1, 0, 0], [1, 0, 0]]

        if isinstance(device_address, list):
            self.acc_gyro_device_address = device_address[0]
            self.mag_device_address = device_address[1]
        else:
            self.acc_gyro_device_address = device_address

        self.accel_add = accel_add
        self.gyro_add = gyro_add
        self.gyro_scale = gyro_scale
        self.mag_add = mag_add
        self.accel_bias = accel_bias
        self.gyro_bias = gyro_bias
        self.mag_bias_a_inv = mag_bias[0]
        self.mag_bias_b = mag_bias[1]

        for value in args:
            self.BUS.write_byte_data(value[0], value[1], value[2])

    def __read_sensor_data(self, device_address: int, addr: list[int, int]) -> int:
        """
        Write data to an address on the bus
        """
        low = self.BUS.read_byte_data(device_address, addr[0])
        high = self.BUS.read_byte_data(device_address, addr[1])
        # bitwise left shift high by 8 bits, then bitwise or on shifted high and low
        value = (high << 8 | low)
        # wrap around overflow
        if value > 2 ** 16 / 2:
            value = value - 2 ** 16
        return value

    def get_bearing(self) -> float:
        """
        Returns bearing value adjusted for declination based on GPS data.
        Will only return a value if magnetometer readings are available
        """
        # todo
        # lat, long = gps.get_coordinates()
        lat = 56
        long = -7

        # rotate upside down as z axis is pointing up (to get positive clockwise angle using right hand rule)
        # rotate 180º as magnetometer is pointing opposite to antenna direction
        bearing = self.z * -1 + 180

        # convert to 0 to 360
        if bearing < 0:
            bearing += 360.0
        if bearing > 360:
            bearing -= 360

        # add declination
        bearing += geomag.declination(lat, long)

        # check if value has overflowed and wrap around
        if bearing > 360:
            bearing -= 360

        return bearing

    def get_adjusted_elevation(self) -> float:
        """
        Returns elevation adjusted to match satellite elevation values of ±90º,
        where 0 is the horizon, +90º is directly overhead and -90º is directly opposite on the other side of Earth
        As satellite elevation is semicircular, there are two unique solutions for a given elevation after conversion
        """

        y = self.y

        if y <= 0:
            y += 90
        else:
            y = -1 * (y - 90)

        return y

    def get_readings(self) -> tuple[float, float, float]:
        """
        Returns unadjusted, smoothed position data
        """
        return self.x, self.y, self.z

    async def stream_readings(self, p) -> None:
        """
        Streams accelerometer and gyroscope data, and magnetometer if present. The data is filtered through a
        Kalman filter to denoise the data
        x, y and z values range from -180 to +180
        """

        # https://vinodembedded.wordpress.com/2021/03/29/how-to-interface-mpu6050-accelerometer-with-raspberry-pi-using-python/
        # https://github.com/TKJElectronics/Example-Sketch-for-IMU-including-Kalman-filter/blob/master/IMU/MPU6050_HMC5883L/MPU6050_HMC5883L.ino
        # https://github.com/niru-5/imusensor/blob/master/imusensor/filters/kalman.py
        # https://github.com/TKJElectronics/KalmanFilter/blob/master/Kalman.cpp
        # https://www.nxp.com/docs/en/application-note/AN3461.pdf
        # https://www.nxp.com/docs/en/application-note/AN4248.pdf

        finish_time = p[2][0]

        kalman_filter_x = kalman.KalmanFilter()
        kalman_filter_z = kalman.KalmanFilter()
        kalman_filter_y = kalman.KalmanFilter()

        kalman_x = 0
        kalman_y = 0
        kalman_z = 0

        a_x = self.__fit_acc_bias(self.__read_sensor_data(self.acc_gyro_device_address, self.accel_add[0]),
                                  *self.accel_bias[0])
        a_y = self.__fit_acc_bias(self.__read_sensor_data(self.acc_gyro_device_address, self.accel_add[1]),
                                  *self.accel_bias[1])
        a_z = self.__fit_acc_bias(self.__read_sensor_data(self.acc_gyro_device_address, self.accel_add[2]),
                                  *self.accel_bias[2])

        timer = time.perf_counter()

        # rotation about x axis is limited to ±90º to allow unique solutions to be calculated
        # roll_x = math.degrees(math.atan2(a_y, a_z))
        # pitch_y = math.degrees(math.atan2(-a_x, math.sqrt((a_y ** 2) + (a_z ** 2))))
        roll_x = np.degrees(np.arctan2(a_y, np.sqrt(a_x ** 2 + a_z ** 2)))
        pitch_y = np.degrees(np.arctan2(-a_x, a_z))

        if self.mag_add is not None:
            m_x = self.__read_sensor_data(self.mag_device_address, self.mag_add[0])
            m_y = self.__read_sensor_data(self.mag_device_address, self.mag_add[1])
            m_z = self.__read_sensor_data(self.mag_device_address, self.mag_add[2])

            h = np.array([[m_x, m_y, m_z]]).T
            h_unit_vec = np.matmul(self.mag_bias_a_inv, h - self.mag_bias_b)
            cal_m_x = h_unit_vec[0][0]
            cal_m_y = h_unit_vec[1][0]
            cal_m_z = h_unit_vec[2][0]

            yaw_y_component = cal_m_z * np.sin(np.radians(kalman_x)) - cal_m_y * np.cos(np.radians(kalman_x))
            yaw_x_component = cal_m_x * np.cos(np.radians(kalman_y)) \
                              + cal_m_y * np.sin(np.radians(kalman_x)) * np.sin(np.radians(kalman_y)) \
                              + cal_m_z * np.cos(np.radians(kalman_x)) * np.sin(np.radians(kalman_y))

            yaw_z = np.degrees(np.arctan2(yaw_y_component, yaw_x_component))

            kalman_filter_z.set_angle(yaw_z)

        kalman_filter_x.set_angle(roll_x)
        kalman_filter_y.set_angle(pitch_y)

        while True:
            # todo
            # if gps.get_time() > finish_time or not shared.running.value:
            #     print('IMU stopped at ' + finish_time)
            #     break

            a_x = self.__fit_acc_bias(self.__read_sensor_data(self.acc_gyro_device_address, self.accel_add[0]),
                                      *self.accel_bias[0])
            a_y = self.__fit_acc_bias(self.__read_sensor_data(self.acc_gyro_device_address, self.accel_add[1]),
                                      *self.accel_bias[1])
            a_z = self.__fit_acc_bias(self.__read_sensor_data(self.acc_gyro_device_address, self.accel_add[2]),
                                      *self.accel_bias[2])

            g_x = (self.__read_sensor_data(self.acc_gyro_device_address, self.gyro_add[0]) + self.gyro_bias[
                0]) * self.gyro_scale
            g_y = (self.__read_sensor_data(self.acc_gyro_device_address, self.gyro_add[1]) + self.gyro_bias[
                1]) * self.gyro_scale
            g_z = (self.__read_sensor_data(self.acc_gyro_device_address, self.gyro_add[2]) + self.gyro_bias[
                2]) * self.gyro_scale

            dt = time.perf_counter() - timer
            timer = time.perf_counter()

            # roll_x = math.degrees(math.atan2(a_y, a_z))
            # pitch_y = math.degrees(math.atan(-a_x / math.sqrt((a_y ** 2) + (a_z ** 2))))
            roll_x = np.degrees(np.arctan2(a_y, np.sqrt(a_x ** 2 + a_z ** 2)))
            pitch_y = np.degrees(np.arctan2(-a_x, a_z))

            if self.mag_add is not None:
                m_x = self.__read_sensor_data(self.mag_device_address, self.mag_add[0])
                m_y = self.__read_sensor_data(self.mag_device_address, self.mag_add[1])
                m_z = self.__read_sensor_data(self.mag_device_address, self.mag_add[2])

                h = np.array([[m_x, m_y, m_z]]).T
                h_unit_vec = np.matmul(self.mag_bias_a_inv, h - self.mag_bias_b)
                cal_m_x = h_unit_vec[0][0]
                cal_m_y = h_unit_vec[1][0]
                cal_m_z = h_unit_vec[2][0]

                yaw_y_component = cal_m_z * np.sin(np.radians(kalman_x)) - cal_m_y * np.cos(np.radians(kalman_x))
                yaw_x_component = cal_m_x * np.cos(np.radians(kalman_y)) \
                                  + cal_m_y * np.sin(np.radians(kalman_x)) * np.sin(np.radians(kalman_y)) \
                                  + cal_m_z * np.cos(np.radians(kalman_x)) * np.sin(np.radians(kalman_y))

                yaw_z = np.degrees(np.arctan2(yaw_y_component, yaw_x_component))

                kalman_z = kalman_filter_z.get_angle(yaw_z, g_z, dt)

            if (pitch_y < -90 and kalman_y > 90) or (pitch_y > 90 and kalman_y < -90):
                kalman_filter_y.set_angle(pitch_y)
                kalman_y = pitch_y
            else:
                kalman_y = kalman_filter_y.get_angle(pitch_y, g_y, dt)

            if abs(kalman_y) < 90:
                g_x = -g_x
                kalman_x = kalman_filter_x.get_angle(roll_x, g_x, dt)

            # reset roll if over 90 degrees
            if kalman_x > 90:
                kalman_x = 90
            elif kalman_x < -90:
                kalman_x = -90

            if kalman_z > 180:
                kalman_z -= 360
            elif kalman_z < -180:
                kalman_z += 360

            self.x = kalman_x
            self.y = kalman_y
            self.z = kalman_z

            # print('x', round(kalman_x), 'y', round(kalman_y), 'z', round(kalman_z))

            await asyncio.sleep(0.05)

    def __fit_acc_bias(self, data, a, b):
        """
        Model function for least squares fitting
        :param data: the value to be adjusted
        :param a: the first fit parameter
        :param b: the second fit parameter
        :return: adjusted data
        """
        return (a * data) + b

    def calibrate_gyroscope(self) -> list[float, float, float]:
        """
        Calibrate gyroscope
        :return: the gyroscope bias
        """
        bias = np.array([0.0, 0.0, 0.0])
        input("Keep the gyroscope still while gathering data. Press enter when ready.")
        for i in range(1000):
            g_x = self.__read_sensor_data(self.acc_gyro_device_address, self.gyro_add[0])
            g_y = self.__read_sensor_data(self.acc_gyro_device_address, self.gyro_add[1])
            g_z = self.__read_sensor_data(self.acc_gyro_device_address, self.gyro_add[2])
            bias += [g_x, g_y, g_z]
            time.sleep(0.02)

        return bias / 1000

    def calibrate_accelerometer(self) -> list[list[float, float], list[float, float], list[float, float]]:
        """
        Calibrate accelerometer
        :return: an array of accelerometer biases
        """
        # https://makersportal.com/blog/calibration-of-an-inertial-measurement-unit-imu-with-raspberry-pi-part-ii
        bias = [[], [], []]
        axes = ['x', 'y', 'z']
        directions = ["upward", "downward", "perpendicular"]  # direction for IMU cal
        for axis_index, axis in enumerate(axes):
            data = [[], [], []]
            for direction_index, direction in enumerate(directions):
                input("Point the " + axis + "-axis " + direction + " and keep still. Press enter when ready.")
                axis_data = []
                while len(axis_data) < 1000:
                    axis_data.append(self.__read_sensor_data(self.acc_gyro_device_address, self.accel_add[axis_index]))

                data[direction_index] = np.array(axis_data)

            # accelerometer data
            x_data = np.append(np.append(data[0], data[1]), data[2])

            # ideal data
            y_data = np.append(np.append(1.0 * np.ones(np.shape(data[0])), -1.0 * np.ones(np.shape(data[1]))),
                               0.0 * np.ones(np.shape(data[2])))

            fit_params, _ = curve_fit(self.__fit_acc_bias, x_data, y_data, maxfev=10000)
            bias[axis_index] = fit_params
        return bias

    def calibrate_magnetometer_approximate(self) -> list[float, float, float]:
        """
        Calibrate magnetometer approximately for hard iron effects to allow more precise calibration to be performed
        https://ozzmaker.com/compass3/
        :return: an array of magnetometer biases
        """
        # magnetometer value is a signed 16 bit value
        limit = 2 ** 16 / 2
        # arrays to store minimum and maximum values
        x_bias = [limit, -limit]
        y_bias = [limit, -limit]
        z_bias = [limit, -limit]

        input("Rotate the magnetometer in all directions. Calibration will complete when the values stabilise.")

        i = 0
        total = 0
        while i < 1000:
            total += 1
            m_x = self.__read_sensor_data(self.acc_gyro_device_address, self.mag_add[0])
            m_y = self.__read_sensor_data(self.acc_gyro_device_address, self.mag_add[1])
            m_z = self.__read_sensor_data(self.mag_device_address, self.mag_add[2])

            if m_x < x_bias[0]:
                i = 0
                x_bias[0] = m_x

            elif m_x > x_bias[1]:
                i = 0
                x_bias[1] = m_x

            if m_y < y_bias[0]:
                i = 0
                y_bias[0] = m_y
            elif m_y > y_bias[1]:
                i = 0
                y_bias[1] = m_y

            if m_z < z_bias[0]:
                i = 0
                z_bias[0] = m_z
            elif m_z > z_bias[1]:
                i = 0
                z_bias[1] = m_z

            i += 1

            print(x_bias)
            print(y_bias)
            print(z_bias)

        # approximate mean value of samples
        x_bias_mean = (x_bias[0] + x_bias[1]) / 2
        y_bias_mean = (y_bias[0] + y_bias[1]) / 2
        z_bias_mean = (z_bias[0] + z_bias[1]) / 2

        return [x_bias_mean, y_bias_mean, z_bias_mean]

    def calibrate_magnetometer_precise(self) -> tuple[Union[list, float, float, float], Any]:
        """
        https://ieeexplore.ieee.org/document/1290055
        https://thepoorengineer.com/en/calibrating-the-magnetometer/
        Calibrate magnetometer in 3 dimensions for hard and soft iron effects with least squares fitting
        :return: a 3x3 matrix and a 3x1 matrix
        """

        input("Rotate the magnetometer smoothly in 3 dimensions, as if around a sphere")

        m_x = []
        m_y = []
        m_z = []

        while len(m_x) < 3000:
            m_x.append(self.__read_sensor_data(self.mag_device_address, self.mag_add[0]))
            m_y.append(self.__read_sensor_data(self.mag_device_address, self.mag_add[1]))
            m_z.append(self.__read_sensor_data(self.mag_device_address, self.mag_add[2]))

        m_x = np.array(m_x)
        m_y = np.array(m_y)
        m_z = np.array(m_z)

        # create Xi matrix
        a1 = m_x ** 2
        a2 = m_y ** 2
        a3 = m_z ** 2
        a4 = 2 * np.multiply(m_y, m_z)
        a5 = 2 * np.multiply(m_x, m_z)
        a6 = 2 * np.multiply(m_x, m_y)
        a7 = 2 * m_x
        a8 = 2 * m_y
        a9 = 2 * m_z
        a10 = np.ones(len(m_x)).T
        d = np.array([a1, a2, a3, a4, a5, a6, a7, a8, a9, a10])

        k = 4
        z = k / 2 - 1

        # Equation 7
        c1 = np.array([[-1, z, z, 0, 0, 0],
                       [z, -1, z, 0, 0, 0],
                       [z, z, -1, 0, 0, 0],
                       [0, 0, 0, -k, 0, 0],
                       [0, 0, 0, 0, -k, 0],
                       [0, 0, 0, 0, 0, -k]])

        # Equation 11
        s = np.matmul(d, d.T)
        s11 = s[:6, :6]
        s12 = s[:6, 6:]
        s21 = s[6:, :6]
        s22 = s[6:, 6:]

        # Equation 15
        tmp = np.matmul(np.linalg.inv(c1), s11 - np.matmul(s12, np.matmul(np.linalg.inv(s22), s12.T)))

        # Get eigenvector of largest eigenvalue
        e_val, e_vect = np.linalg.eig(tmp)
        u1 = e_vect[:, np.argmax(e_val)]

        # Equation 13
        u2 = np.matmul(-np.matmul(np.linalg.inv(s22), s21), u1)

        # Get
        u = np.concatenate([u1, u2]).T

        # A * A^-1
        q = np.array([[u[0], u[5], u[4]],
                      [u[5], u[1], u[3]],
                      [u[4], u[3], u[2]]])

        n = np.array([[u[6]],
                      [u[7]],
                      [u[8]]])

        d = u[9]

        q_inv = np.linalg.inv(q)
        b = -np.dot(q_inv, n)
        a_inv = np.real(1 / np.sqrt(np.dot(n.T, np.dot(q_inv, n)) - d) * linalg.sqrtm(q))

        return a_inv, b

import asyncio
import time
from typing import Union, Any
import smbus
import numpy as np
import geomag
from scipy.optimize import curve_fit
from scipy import linalg
import gps
import kalman
import shared


class IMU(object):
    """IMU object to calculate orientation from sensors"""

    def __init__(self, device_address: Union[list, int], gyro_scale: float, accel_address:list, gyro_address:list, mag_address:list = None,
                 accel_bias:list = None, gyro_bias:list = None, mag_bias = None, *args: list[int, int, int]):
        """Initialise IMU using accelerometer and (optionally) magnetometer addresses

        :param device_address: The accelerometer and gyroscope bus address
        :param gyro_scale: The factor to scale gyroscope data by (from datasheet)
        :param accel_address: The accelerometer's x, y and z addresses
        :param gyro_address: The gyroscope's x, y and z addresses
        :param mag_address: The Magnetometer's x, y and z addresses
        :param accel_bias: The accelerometer's bias
        :param gyro_bias: The gyroscope's bias
        :param mag_bias: The magnetometer bias
        :param args: Miscellaneous setup addresses with device address and value
        """

        self._bus = smbus.SMBus(1)
        self._roll = 0
        self._pitch = 0
        self._yaw = 0

        if accel_bias is None:
            accel_bias = [[1, 0], [1, 0], [1, 0]]

        if gyro_bias is None:
            gyro_bias = [0, 0, 0]

        if mag_bias is None:
            mag_bias = [[1, 0, 0], [1, 0, 0], [1, 0, 0]]

        if isinstance(device_address, list):
            self._acc_gyro_device_address = device_address[0]
            self._mag_device_address = device_address[1]
        else:
            self._acc_gyro_device_address = device_address

        self._accel_address = accel_address
        self._gyro_address = gyro_address
        self._gyro_scale = gyro_scale
        self._mag_address = mag_address
        self._accel_bias = accel_bias
        self._gyro_bias = gyro_bias
        self._mag_bias_a_inv = mag_bias[0]
        self._mag_bias_b = mag_bias[1]

        for value in args:
            self._bus.write_byte_data(value[0], value[1], value[2])

    @property
    def roll(self) -> float:
        """Returns the roll"""
        return self._roll

    @property
    def pitch(self) -> float:
        """Returns the pitch"""
        return self._pitch

    @property
    def yaw(self) -> float:
        """Returns the yaw"""
        return self._yaw

    def __read_sensor_data(self, device_address: int, address: list[int, int]) -> int:
        """Read data from an address

        Read data from a device on the bus in 16 bit two's complement
        :param device_address: The device to read from
        :param address: The high (MSB) and low (LSB) addresses for the reading
        """
        low = self._bus.read_byte_data(device_address, address[0])
        high = self._bus.read_byte_data(device_address, address[1])
        # Bitwise left shift msb/high by 8 bits, then bitwise or on shifted high and lsb/low
        # Combines low and high together
        # eg high = abc, low = 01234567
        # abc << 8 = 00000000abc => 00000000abc | 01234567 = 01234567abc
        value = (high << 8 | low)
        # Wrap around overflow
        if value > 2 ** 16 / 2:
            value = value - 2 ** 16
        return value

    @property
    def bearing(self) -> float:
        """Gets the current bearing

        Returns bearing value adjusted for declination based on GPS data.
        Will only return a value if magnetometer readings are available
        """
        lat = gps.gps.location.latitude.degrees
        long = gps.gps.location.longitude.degrees

        # Rotate upside down as z axis is pointing up (to get positive clockwise angle using right hand rule)
        # Rotate 180º as magnetometer is pointing opposite to antenna direction
        bearing = self._yaw * -1 + 180

        # Convert to 0 to 360
        if bearing < 0:
            bearing += 360.0
        elif bearing > 360:
            bearing -= 360

        # Add declination
        bearing += geomag.declination(lat, long)

        # Check if value has overflowed and wrap around
        if bearing < 0:
            bearing += 360.0
        elif bearing > 360:
            bearing -= 360

        return bearing

    @property
    def adjusted_elevation(self) -> float:
        """Gets the satellite adjusted elevation value

        Returns elevation adjusted to match satellite elevation values of ±90º,
        where 0 is the horizon, +90º is directly overhead and -90º is directly opposite on the other side of Earth
        As satellite elevation is semicircular, there are two unique solutions for a given elevation after conversion
        """

        y = self._pitch

        if y <= 0:
            y += 90
        else:
            y = -1 * (y - 90)

        return y

    @property
    def readings(self) -> tuple[float, float, float]:
        """Get the current roll, pitch and yaw

        Returns unadjusted, smoothed position data
        :return array of floats for roll, pitch and yaw
        """
        return self._roll, self._pitch, self._yaw

    def __get_raw_gyroscope(self) -> tuple[float, float, float]:
        """Get gyroscope readings

        Teh gyroscope readings are calibrated and scaled
        :return: the gyroscope readings
        """
        g_x = (self.__read_sensor_data(self._acc_gyro_device_address, self._gyro_address[0]) + self._gyro_bias[
            0]) * self._gyro_scale
        g_y = (self.__read_sensor_data(self._acc_gyro_device_address, self._gyro_address[1]) + self._gyro_bias[
            1]) * self._gyro_scale
        g_z = (self.__read_sensor_data(self._acc_gyro_device_address, self._gyro_address[2]) + self._gyro_bias[
            2]) * self._gyro_scale

        return g_x, g_y, g_z

    def __get_raw_roll_pitch(self) -> tuple[float, float]:
        """Get roll and pitch from accelerometer

        The accelerometer values are calibrated before the roll and pitch is calculated
        :return: the roll and pitch
        """
        a_x = self.__fit_acc_bias(self.__read_sensor_data(self._acc_gyro_device_address, self._accel_address[0]),
                                  *self._accel_bias[0])
        a_y = self.__fit_acc_bias(self.__read_sensor_data(self._acc_gyro_device_address, self._accel_address[1]),
                                  *self._accel_bias[1])
        a_z = self.__fit_acc_bias(self.__read_sensor_data(self._acc_gyro_device_address, self._accel_address[2]),
                                  *self._accel_bias[2])

        roll_x = np.rad2deg(np.arctan2(a_y, np.sqrt(a_x ** 2 + a_z ** 2)))
        pitch_y = np.rad2deg(np.arctan2(-a_x, a_z))
        return  roll_x, pitch_y

    def __get_raw_yaw(self, kalman_x: float, kalman_y: float) -> float:
        """Get yaw from magnetometer

        The magnetometer reading is calibrated and pitch and roll are Kalman filtered
        :param kalman_x: the kalman filtered roll
        :param kalman_y: the kalman filtered pitch
        :return: the yaw
        """
        m_x = self.__read_sensor_data(self._mag_device_address, self._mag_address[0])
        m_y = self.__read_sensor_data(self._mag_device_address, self._mag_address[1])
        m_z = self.__read_sensor_data(self._mag_device_address, self._mag_address[2])

        h = np.array([[m_x, m_y, m_z]]).T
        h_unit_vec = np.matmul(self._mag_bias_a_inv, h - self._mag_bias_b)
        cal_m_x = h_unit_vec[0][0]
        cal_m_y = h_unit_vec[1][0]
        cal_m_z = h_unit_vec[2][0]

        yaw_y_component = cal_m_z * np.sin(np.radians(kalman_x)) - cal_m_y * np.cos(np.radians(kalman_x))
        yaw_x_component = cal_m_x * np.cos(np.radians(kalman_y)) \
                          + cal_m_y * np.sin(np.radians(kalman_x)) * np.sin(np.radians(kalman_y)) \
                          + cal_m_z * np.cos(np.radians(kalman_x)) * np.sin(np.radians(kalman_y))

        yaw_z = np.rad2deg(np.arctan2(yaw_y_component, yaw_x_component))

        return yaw_z


    async def stream_readings(self, passes: list) -> None:
        """Stream roll, pitch and yaw

        Streams accelerometer and gyroscope data, and magnetometer if present. The data is filtered through a
        Kalman filter to increase accuracy
        x, y and z values range from -180 to +180
        :param passes: the satellite rise and set time
        """

        # https://vinodembedded.wordpress.com/2021/03/29/how-to-interface-mpu6050-accelerometer-with-raspberry-pi-using-python/
        # https://github.com/TKJElectronics/Example-Sketch-for-IMU-including-Kalman-filter/blob/master/IMU/MPU6050_HMC5883L/MPU6050_HMC5883L.ino
        # https://github.com/niru-5/imusensor/blob/master/imusensor/filters/kalman.py
        # https://github.com/TKJElectronics/KalmanFilter/blob/master/Kalman.cpp
        # https://www.nxp.com/docs/en/application-note/AN3461.pdf
        # https://www.nxp.com/docs/en/application-note/AN4248.pdf

        finish_time = passes[2][0]

        kalman_x = 0
        kalman_y = 0
        kalman_z = 0

        roll_x, pitch_y = self.__get_raw_roll_pitch()

        timer = time.perf_counter()

        kalman_filter_x = kalman.KalmanFilter(roll_x)
        kalman_filter_y = kalman.KalmanFilter(pitch_y)
        kalman_filter_z = None

        if self._mag_address is not None:
            kalman_filter_z = kalman.KalmanFilter(self.__get_raw_yaw(roll_x, pitch_y))

        while True:
            if gps.gps.time > finish_time or not shared.running.value:
                print('IMU stopped at ' + str(finish_time))
                break

            roll_x, pitch_y = self.__get_raw_roll_pitch()

            g_x, g_y, g_z = self.__get_raw_gyroscope()

            dt = time.perf_counter() - timer
            timer = time.perf_counter()

            if self._mag_address is not None:
                yaw_z = self.__get_raw_yaw(kalman_x, kalman_y)
                # Reset kalman filter when value wraps around the 180º value as the kalman filter doesn't
                # understand wrapping
                if (yaw_z < -90 and kalman_z > 90) or (yaw_z > 90 and kalman_z < -90):
                    kalman_filter_z.set_angle(yaw_z)
                    kalman_z = yaw_z
                else:
                    kalman_z = kalman_filter_z.get_angle(yaw_z, g_z, dt)

            if (pitch_y < -90 and kalman_y > 90) or (pitch_y > 90 and kalman_y < -90):
                kalman_filter_y.set_angle(pitch_y)
                kalman_y = pitch_y
            else:
                kalman_y = kalman_filter_y.get_angle(pitch_y, g_y, dt)

            # If pitch is on upper quadrants of circle (ie not upside down), calculate roll
            # Restricts roll to ±90º to get a single unique solution for IMU orientation
            if abs(kalman_y) < 90:
                kalman_x = kalman_filter_x.get_angle(roll_x, g_x, dt)

            # Reset roll if over 90 degrees
            if kalman_x > 90:
                kalman_x = 90
            elif kalman_x < -90:
                kalman_x = -90

            # Ensure yaw is -180<=y<=180
            if kalman_z > 180:
                kalman_z -= 360
            elif kalman_z < -180:
                kalman_z += 360

            self._roll = kalman_x
            self._pitch = kalman_y
            self._yaw = kalman_z

            # print(self.roll, self.pitch, self.yaw)

            await asyncio.sleep(0.05)

    def __fit_acc_bias(self, data: Union[np.array, float], a: float, b: float) -> Union[np.array, float]:
        """Model function for least squares fitting

        :param data: the value to be adjusted
        :param a: the first fit parameter
        :param b: the second fit parameter
        :return: adjusted data
        """
        return (a * data) + b

    def calibrate_gyroscope(self) -> list[float, float, float]:
        """Calibrate gyroscope

        :return: the gyroscope bias
        """
        bias = np.array([0.0, 0.0, 0.0])
        input("Keep the gyroscope still while gathering data. Press enter when ready.")
        for i in range(1000):
            g_x = self.__read_sensor_data(self._acc_gyro_device_address, self._gyro_address[0])
            g_y = self.__read_sensor_data(self._acc_gyro_device_address, self._gyro_address[1])
            g_z = self.__read_sensor_data(self._acc_gyro_device_address, self._gyro_address[2])
            bias += [g_x, g_y, g_z]
            time.sleep(0.02)

        return bias / 1000

    def calibrate_accelerometer(self) -> list[list[float, float], list[float, float], list[float, float]]:
        """Calibrate accelerometer

        :return: an array of accelerometer biases
        """
        # https://makersportal.com/blog/calibration-of-an-inertial-measurement-unit-imu-with-raspberry-pi-part-ii
        bias = [[], [], []]
        axes = ['x', 'y', 'z']
        # Direction for IMU calibration
        directions = ["upward", "downward", "perpendicular"]
        for axis_index, axis in enumerate(axes):
            data = [[], [], []]
            for direction_index, direction in enumerate(directions):
                input("Point the " + axis + "-axis " + direction + " and keep still. Press enter when ready.")
                axis_data = []
                while len(axis_data) < 1000:
                    axis_data.append(self.__read_sensor_data(self._acc_gyro_device_address, self._accel_address[axis_index]))

                data[direction_index] = np.array(axis_data)

            # Accelerometer data
            x_data = np.append(np.append(data[0], data[1]), data[2])

            # Ideal data
            y_data = np.append(np.append(1.0 * np.ones(np.shape(data[0])), -1.0 * np.ones(np.shape(data[1]))),
                               0.0 * np.ones(np.shape(data[2])))

            fit_params, _ = curve_fit(self.__fit_acc_bias, x_data, y_data, maxfev=10000)
            bias[axis_index] = fit_params
        return bias

    def calibrate_magnetometer_approximate(self) -> list[float, float, float]:
        """Approximately calibrate magnetometer

        Calibrate magnetometer approximately for hard iron effects
        https://ozzmaker.com/compass3/
        :return: an array of magnetometer biases
        """
        # Magnetometer value is a signed 16 bit value
        limit = 2 ** 16 / 2
        # Lists to store minimum and maximum values
        x_bias = [limit, -limit]
        y_bias = [limit, -limit]
        z_bias = [limit, -limit]

        input("Rotate the magnetometer in all directions. Calibration will complete when the values stabilise.")

        i = 0
        total = 0
        while i < 1000:
            total += 1
            m_x = self.__read_sensor_data(self._acc_gyro_device_address, self._mag_address[0])
            m_y = self.__read_sensor_data(self._acc_gyro_device_address, self._mag_address[1])
            m_z = self.__read_sensor_data(self._mag_device_address, self._mag_address[2])

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

        # Approximate mean value of samples
        x_bias_mean = (x_bias[0] + x_bias[1]) / 2
        y_bias_mean = (y_bias[0] + y_bias[1]) / 2
        z_bias_mean = (z_bias[0] + z_bias[1]) / 2

        return [x_bias_mean, y_bias_mean, z_bias_mean]

    def calibrate_magnetometer_precise(self) -> tuple[Union[list, float, float, float], Any]:
        """Precisely calibrate magnetometer

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
            m_x.append(self.__read_sensor_data(self._mag_device_address, self._mag_address[0]))
            m_y.append(self.__read_sensor_data(self._mag_device_address, self._mag_address[1]))
            m_z.append(self.__read_sensor_data(self._mag_device_address, self._mag_address[2]))

        m_x = np.array(m_x)
        m_y = np.array(m_y)
        m_z = np.array(m_z)

        # Create Xi matrix
        x1 = m_x ** 2
        x2 = m_y ** 2
        x3 = m_z ** 2
        x4 = 2 * (m_y * m_z)
        x5 = 2 * (m_x * m_z)
        x6 = 2 * (m_x * m_y)
        x7 = 2 * m_x
        x8 = 2 * m_y
        x9 = 2 * m_z
        x10 = np.ones(len(m_x)).T
        d = np.array([x1, x2, x3, x4, x5, x6, x7, x8, x9, x10])

        # k = 4 for ellipsoids that are at least half as wide as they are tall
        # For values of k != 4, the same paper shows how an iterative method can be used to find k
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
        lhs = np.matmul(np.linalg.inv(c1), s11 - np.matmul(s12, np.matmul(np.linalg.inv(s22), s12.T)))

        # Get eigenvector of largest eigenvalue
        e_val, e_vector = np.linalg.eig(lhs)
        v1 = e_vector[:, np.argmax(e_val)]

        # Equation 13
        v2 = np.matmul(-np.matmul(np.linalg.inv(s22), s21), v1)

        # Right eigenvector (column) to left eigenvector (row)
        u = np.concatenate([v1, v2]).T

        # A * A^-1 - First term of quadric form
        q = np.array([[u[0], u[5], u[4]],
                      [u[5], u[1], u[3]],
                      [u[4], u[3], u[2]]])

        # Second term of quadric form
        n = np.array([[u[6]],
                      [u[7]],
                      [u[8]]])

        # Third term of quadric form
        d = u[9]

        q_inv = np.linalg.inv(q)
        b = -np.dot(q_inv, n)
        a_inv = np.real(1 / np.sqrt(np.dot(n.T, np.dot(q_inv, n)) - d) * linalg.sqrtm(q))

        return a_inv, b

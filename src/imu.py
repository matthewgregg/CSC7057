import asyncio
import time
import smbus
import math
import numpy as np
import geomag
from scipy.optimize import curve_fit
import gps
import kalman


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
        self.mag_bias = mag_bias

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
        Returns yaw value adjusted for declination based on GPS data.
        Will only return a value if magnetometer readings and available
        """
        # TODO revert this when not testing
        # lat, long = gps.get_coordinates()
        lat = 54.5
        long = -6

        bearing = self.z + geomag.declination(lat, long)

        if bearing < 0:
            bearing += 360

        return bearing

    def get_adjusted_elevation(self) -> float:
        """
        Returns elevation adjusted to match satellite elevation values of ±90º,
        where 0 is the horizon, +90º is directly overhead and -90º is directly opposite on the dark side of Earth
        """

        adjusted_x = 0

        if self.x <= 0:
            adjusted_x += 90
        else:
            adjusted_x = -1 * (self.x - 90)

        return adjusted_x

    def get_readings(self) -> tuple[float, float, float]:
        """
        Returns unadjusted, smoothed position data
        """
        return self.x, self.y, self.z

    async def stream_readings(self, p) -> None:
        """
        Streams accelerometer and gyroscope data, and magnetometer if present. The data is filtered through a
        Kalman filter to denoise the data
        """

        # https://vinodembedded.wordpress.com/2021/03/29/how-to-interface-mpu6050-accelerometer-with-raspberry-pi-using-python/
        # https://github.com/rocheparadox/Kalman-Filter-Python-for-mpu6050/blob/master/AngleOMeter.py
        # https://github.com/niru-5/imusensor/blob/master/imusensor/filters/kalman.py

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

        # rotation about y axis is limited to ±90º to allow unique solutions to be calculated
        roll_x = math.degrees(math.atan2(a_y, a_z))
        pitch_y = math.degrees(math.atan(-a_x / math.sqrt((a_y ** 2) + (a_z ** 2))))

        if self.mag_add is not None:
            m_x = self.__read_sensor_data(self.mag_device_address, self.mag_add[0])
            m_y = self.__read_sensor_data(self.mag_device_address, self.mag_add[1])
            m_z = self.__read_sensor_data(self.mag_device_address, self.mag_add[2])

            # yaw_x_component = m_x * np.cos(pitch_y) + m_z * np.sin(pitch_y)
            # yaw_y_component = m_x * np.sin(roll_x) * np.sin(pitch_y) + m_y * np.cos(roll_x) \
            #                   + m_z * np.sin(roll_x) * np.cos(pitch_y)

            yaw_y_component = m_z * np.sin(roll_x) - m_y * np.cos(roll_x)
            yaw_x_component = m_x * np.cos(pitch_y) + m_y * np.sin(roll_x) * np.sin(pitch_y) + m_z * np.cos(
                roll_x) * np.sin(pitch_y)

            yaw_z = np.degrees(np.arctan2(yaw_y_component, yaw_x_component))

            kalman_filter_z.set_angle(yaw_z)

        kalman_filter_x.set_angle(roll_x)
        kalman_filter_y.set_angle(pitch_y)

        while True:
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

            roll_x = math.degrees(math.atan2(a_y, a_z))
            pitch_y = math.degrees(math.atan(-a_x / math.sqrt((a_y ** 2) + (a_z ** 2))))

            if self.mag_add is not None:
                m_x = self.__read_sensor_data(self.mag_device_address, self.mag_add[0])
                m_y = self.__read_sensor_data(self.mag_device_address, self.mag_add[1])
                m_z = self.__read_sensor_data(self.mag_device_address, self.mag_add[2])

                # https://ozzmaker.com/compass2/
                # yaw_x_component = m_x * np.cos(pitch_y) + m_z * np.sin(pitch_y)
                # yaw_y_component = m_x * np.sin(roll_x) * np.sin(pitch_y) + m_y * np.cos(roll_x) \
                #                   + m_z * np.sin(roll_x) * np.cos(pitch_y)

                yaw_y_component = m_z * np.sin(roll_x) - m_y * np.cos(roll_x)
                yaw_x_component = m_x * np.cos(pitch_y) \
                                  + m_y * np.sin(roll_x) * np.sin(pitch_y) \
                                  + m_z * np.cos(roll_x) * np.sin(pitch_y)

                yaw_z = np.degrees(np.arctan2(yaw_y_component, yaw_x_component))

                kalman_z = kalman_filter_z.get_angle(yaw_z, g_z, dt)

            if (roll_x < -90 and kalman_x > 90) or (roll_x > 90 and kalman_x < -90):
                kalman_filter_x.set_angle(roll_x)
                kalman_x = roll_x
            else:
                kalman_x = kalman_filter_x.get_angle(roll_x, g_x, dt)

            if abs(kalman_x) > 90:
                g_y = -g_y
                kalman_y = kalman_filter_y.get_angle(pitch_y, g_y, dt)

            self.x = kalman_x
            self.y = kalman_y
            self.z = kalman_z

            print(roll_x, pitch_y, yaw_z)

            await asyncio.sleep(0.5)

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
        for i in range(500):
            g_x = self.__read_sensor_data(self.acc_gyro_device_address, self.accel_add[0])
            g_y = self.__read_sensor_data(self.acc_gyro_device_address, self.accel_add[1])
            g_z = self.__read_sensor_data(self.mag_device_address, self.accel_add[2])
            bias += [g_x, g_y, g_z]
            time.sleep(0.02)

        return bias / len(bias)

    def calibrate_accelerometer(self) -> list[list[float, float], list[float, float], list[float, float]]:
        """
        Calibrate accelerometer
        :return: an array of accelerometer biases
        """
        # https://makersportal.com/blog/calibration-of-an-inertial-measurement-unit-imu-with-raspberry-pi-part-ii#gyro
        sample_size = 1000
        bias = [[], [], []]
        axes = ['x', 'y', 'z']
        axes_addresses = [*self.accel_add]
        device_address = [self.acc_gyro_device_address, self.acc_gyro_device_address, self.mag_device_address]
        directions = ["up", "down", "perpendicular"]
        axis_coefficients = [1, -1, 0]
        for axis_index, axis in enumerate(axes):
            data = [[], [], []]
            for direction_index, direction in enumerate(directions):
                input("Point the " + axis + "-axis " + direction + " and keep still. Press enter when ready.")
                axis_data = []
                while len(axis_data) < sample_size:
                    axis_data.append(self.__read_sensor_data(device_address[axis_index], axes_addresses[axis_index]))

                data[direction_index] = np.array(axis_data)

            x_data = data[0] + data[1] + data[2]
            # y_data = np.sum(axis_coefficients * np.ones(np.shape(data)), axis=0)
            y_data = 1 * np.ones(np.shape(data[0])) + -1 * np.ones(np.shape(data[1])) + 0 * np.ones(np.shape(data[2]))

            fit_params, _ = curve_fit(self.__fit_acc_bias, x_data, y_data, maxfev=10000)
            bias[axis_index] = fit_params
        return bias

    def accel_cal(self):
        print("-" * 50)
        print("Accelerometer Calibration")
        mpu_offsets = [[], [], []]  # offset array to be printed
        axis_vec = ['z', 'y', 'x']  # axis labels
        cal_directions = ["upward", "downward", "perpendicular to gravity"]  # direction for IMU cal
        cal_indices = [2, 1, 0]  # axis indices
        for qq, ax_qq in enumerate(axis_vec):
            ax_offsets = [[], [], []]
            print("-" * 50)
            for direc_ii, direc in enumerate(cal_directions):
                input("-" * 8 + " Press Enter and Keep IMU Steady to Calibrate the Accelerometer with the -" +
                      ax_qq + "-axis pointed " + direc)
                mpu_array = []
                while len(mpu_array) < 1000:
                    try:
                        ax = self.__read_sensor_data(self.acc_gyro_device_address, self.accel_add[0])
                        ay = self.__read_sensor_data(self.acc_gyro_device_address, self.accel_add[1])
                        az = self.__read_sensor_data(self.mag_device_address, self.accel_add[2])
                        mpu_array.append([ax, ay, az])  # append to array
                    except:
                        continue
                ax_offsets[direc_ii] = np.array(mpu_array)[:, cal_indices[qq]]  # offsets for direction

            # Use three calibrations (+1g, -1g, 0g) for linear fit
            popts, _ = curve_fit(self.__fit_acc_bias, np.append(np.append(ax_offsets[0],
                                                                          ax_offsets[1]), ax_offsets[2]),
                                 np.append(np.append(1.0 * np.ones(np.shape(ax_offsets[0])),
                                                     -1.0 * np.ones(np.shape(ax_offsets[1]))),
                                           0.0 * np.ones(np.shape(ax_offsets[2]))),
                                 maxfev=10000)
            mpu_offsets[cal_indices[qq]] = popts  # place slope and intercept in offset array
        print('Accelerometer Calibrations Complete')
        return mpu_offsets

    def calibrate_magnetometer_approximate(self) -> list[float, float, float]:
        """
        Calibrate magnetometer approximately for hard iron effects to allow more precise calibration to be performed
        https://ozzmaker.com/compass3/
        :return: an array of magnetometer biases
        """
        x_bias = [1e6, -1e6]
        y_bias = [1e6, -1e6]
        z_bias = [1e6, -1e6]

        input("Rotate the magnetometer in all directions. Calibration will complete when the values stabilise.")

        count = 0
        while count < 100:
            m_x = self.__read_sensor_data(self.acc_gyro_device_address, self.mag_add[0])
            m_y = self.__read_sensor_data(self.acc_gyro_device_address, self.mag_add[1])
            m_z = self.__read_sensor_data(self.mag_device_address, self.mag_add[2])

            if m_x < x_bias[0]:
                count = 0
                x_bias[0] = m_x
            elif m_x > x_bias[1]:
                count = 0
                x_bias[1] = m_x

            if m_y < y_bias[0]:
                count = 0
                y_bias[0] = m_y
            elif m_y > y_bias[1]:
                count = 0
                y_bias[1] = m_y

            if m_z < z_bias[0]:
                count = 0
                z_bias[0] = m_z
            elif m_z > z_bias[1]:
                count = 0
                z_bias[1] = m_z

            count += 1

            print(x_bias)
            print(y_bias)
            print(z_bias)

        # approximate mean value of samples
        x_bias_mean = (x_bias[0] + x_bias[1]) / 2
        y_bias_mean = (y_bias[0] + y_bias[1]) / 2
        z_bias_mean = (z_bias[0] + z_bias[1]) / 2

        return [x_bias_mean, y_bias_mean, z_bias_mean]

    def calibrate_magnetometer_precise(self, approximate_bias) -> list[list[float, float, float],
                                                                       list[float, float, float],
                                                                       list[float, float, float]]:
        """
        Calibrate magnetometer x and y axes for hard and soft iron effects
        The z axis bias is not calculated for simplification. The z axis is not used to calculate bearing
        :return: an array of magnetometer biases
        """
        # TODO calculate calibration matrix here

        cal = [[0, 0, 0],
               [0, 0, 0],
               [0, 0, 0]]

        # adjust values to compensate for approximate calibration
        cal[0][2] -= (approximate_bias[0] * cal[0][0] + approximate_bias[1] * cal[0][1])
        cal[1][2] -= (approximate_bias[0] * cal[1][0] + approximate_bias[1] * cal[1][1])

        return cal

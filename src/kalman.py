# https://blog.tkjelectronics.dk/2012/09/a-practical-approach-to-kalman-filter-and-how-to-implement-it/

class KalmanFilter:
    """Kalman filter object to predict the state an angle"""
    # Process noise - accelerometer variance
    Q_ANGLE = 0.001
    # Process noise - bias variance
    Q_GYRO_BIAS = 0.003
    # Measurement noise
    R_MEASURE = 0.03

    def __init__(self, angle: float):
        """Initialise a Kalman filter with an initial angle

        The initial angle reduces the settling time of the filtered angle
        :param angle: The initial angle
        """
        if not isinstance(angle, (float, int)):
            raise TypeError
        # Angle
        self._angle = angle
        # Bias
        self._bias = 0.0
        # Angular velocity from gyroscope (unbiased rate)
        self._angular_velocity = 0.0
        # Error covariance matrix
        self._p = [[0.0, 0.0], [0.0, 0.0]]

    def set_angle(self, angle: float) -> None:
        """Set Kalman angle

        Resets the Kalman filtered angle
        :param angle: The angle to set
        :return: None
        """
        self._angle = angle


    def get_angle(self, new_angle: float, new_ang_vel: float, dt: float) -> float:
        """Get Kalman angle

        Get the Kalman filtered angle
        :param new_angle: The current angle
        :param new_ang_vel: The current angular velocity
        :param dt: The time difference since the last update
        :return: The filtered angle
        """
        if not isinstance(new_angle, (float, int)) \
                or not isinstance(new_ang_vel, (float, int)) \
                or not isinstance(dt, (float, int)):
            raise TypeError

        # Predict state
        self._angular_velocity = new_ang_vel - self._bias
        # Sum with integral of predicted rate (ie change in angle)
        self._angle += dt * self._angular_velocity

        # Predict error covariance matrix based on previous
        self._p[0][0] += dt * (dt * self._p[1][1] - self._p[0][1] - self._p[1][0] + self.Q_ANGLE)
        self._p[0][1] -= dt * self._p[1][1]
        self._p[1][0] -= dt * self._p[1][1]
        self._p[1][1] += dt * self.Q_GYRO_BIAS

        # Calculate innovation, the difference between the measurement and the predicted state
        y = new_angle - self._angle

        # Calculate innovation covariance
        # Prediction of the confidence in measurement based on predicted error covariance matrix
        s = self._p[0][0] + self.R_MEASURE

        # Calculate kalman gain, prediction of the confidence in innovation
        k = [0, 0]
        k[0] = self._p[0][0] / s
        k[1] = self._p[1][0] / s

        # Update estimate of current state
        self._angle += k[0] * y
        self._bias += k[1] * y

        p00_orig = self._p[0][0]
        p01_orig = self._p[0][1]

        # Decrease error covariance matrix, as the error of the estimate of the state decreased.
        self._p[0][0] -= k[0] * p00_orig
        self._p[0][1] -= k[0] * p01_orig
        self._p[1][0] -= k[1] * p00_orig
        self._p[1][1] -= k[1] * p01_orig

        return self._angle

# https://blog.tkjelectronics.dk/2012/09/a-practical-approach-to-kalman-filter-and-how-to-implement-it/

class KalmanFilter:
    # process noise - accelerometer variance
    Q_ANGLE = 0.001
    # process noise - bias variance
    Q_GYRO_BIAS = 0.003
    # measurement noise
    R_MEASURE = 0.03

    def __init__(self, angle):
        if not isinstance(angle, (float, int)):
            raise TypeError
        # angle
        self.angle = angle
        # bias
        self.bias = 0.0
        # angular velocity from gyro - unbiased rate
        self.ang_vel = 0.0
        # error covariance matrix
        self.p = [[0.0, 0.0], [0.0, 0.0]]

    def set_angle(self, angle):
        self.angle = angle


    def get_angle(self, new_angle, new_ang_vel, dt):
        if not isinstance(new_angle, (float, int)) \
                or not isinstance(new_ang_vel, (float, int)) \
                or not isinstance(dt, (float, int)):
            raise TypeError
        # predict state
        self.ang_vel = new_ang_vel - self.bias
        # sum with integral of predicted rate (ie change in angle)
        self.angle += dt * self.ang_vel

        # predict error covariance matrix based on previous
        self.p[0][0] += dt * (dt * self.p[1][1] - self.p[0][1] - self.p[1][0] + self.Q_ANGLE)
        self.p[0][1] -= dt * self.p[1][1]
        self.p[1][0] -= dt * self.p[1][1]
        self.p[1][1] += dt * self.Q_GYRO_BIAS

        # calculate innovation, the difference between the measurement and the predicted state
        y = new_angle - self.angle

        # calculate innovation covariance
        # prediction of the confidence in measurement based on predicted error covariance matrix
        s = self.p[0][0] + self.R_MEASURE

        # calculate kalman gain, prediction of the confidence in innovation
        k = [0, 0]
        k[0] = self.p[0][0] / s
        k[1] = self.p[1][0] / s

        # update estimate of current state
        self.angle += k[0] * y
        self.bias += k[1] * y

        p00_orig = self.p[0][0]
        p01_orig = self.p[0][1]

        # decrease error covariance matrix, as the error of the estimate of the state decreased.
        self.p[0][0] -= k[0] * p00_orig
        self.p[0][1] -= k[0] * p01_orig
        self.p[1][0] -= k[1] * p00_orig
        self.p[1][1] -= k[1] * p01_orig

        return self.angle

# https://blog.tkjelectronics.dk/2012/09/a-practical-approach-to-kalman-filter-and-how-to-implement-it/

class KalmanFilter:
    # error
    Q_ANGLE = 0.001
    # drift error
    Q_GYRO_BIAS = 0.003
    # measurement error
    R_MEASURE = 0.03

    def __init__(self):
        # angle
        self.angle = 0.0
        # bias
        self.bias = 0.0
        # angular velocity from gyro
        self.rate = 0.0
        # covariance
        self.p = [[0.0, 0.0], [0.0, 0.0]]

    def get_angle(self, new_angle, new_rate, dt):
        self.rate = new_rate - self.bias
        self.angle += dt * self.rate

        self.p[0][0] += dt * (dt * self.p[1][1] - self.p[0][1] - self.p[1][0] + self.Q_ANGLE)
        self.p[0][1] -= dt * self.p[1][1]
        self.p[1][0] -= dt * self.p[1][1]
        self.p[1][1] += dt * self.Q_GYRO_BIAS

        s = self.p[0][0] + self.R_MEASURE

        k = [0.0, 0.0]
        k[0] = self.p[0][0] / s
        k[1] = self.p[1][0] / s

        y = new_angle - self.angle

        self.angle += k[0] * y
        self.bias += k[1] * y

        p00_temp = self.p[0][0]
        p01_temp = self.p[0][1]

        self.p[0][0] -= k[0] * p00_temp
        self.p[0][1] -= k[0] * p01_temp
        self.p[1][0] -= k[1] * p00_temp
        self.p[1][1] -= k[1] * p01_temp

        return self.angle

    def set_angle(self, angle):
        self.angle = angle
from unittest import TestCase
import kalman

class TestKalmanFilter(TestCase):

    def setUp(self):
        self.angle = 1
        self.k = kalman.KalmanFilter(self.angle)

    def test_constructor_valid(self):
        angle = 1
        k2 = kalman.KalmanFilter(angle)
        self.assertEqual(k2.get_angle(0, 0, 0), angle)

    def test_constructor_invalid(self):
        angle = True
        k2 = kalman.KalmanFilter(angle)
        self.assertRaises(TypeError, k2.get_angle(0, 0, 0))

    def test_set_get_angle_valid(self):
        angle = 2
        rate = 1
        dt = 1
        ans = 3
        self.k.set_angle(angle)
        self.assertEqual(self.k.get_angle(angle+1, rate, dt), ans, 3)

    def test_set_get_angle_invalid(self):
        angle = True
        rate = True
        dt = True
        self.assertRaises(TypeError, self.k.get_angle(angle, 0, 0))
        self.assertRaises(TypeError, self.k.get_angle(0, rate, 0))
        self.assertRaises(TypeError, self.k.get_angle(0, 0, dt))

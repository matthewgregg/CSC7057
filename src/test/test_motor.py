import asyncio
import time
from unittest import TestCase
import motor


class Test(TestCase):
    max = 90
    motor1 = motor.Motor([24, 25, 8, 7], 1, max)

    def test_constructor_valid(self):
        pins = [24, 25, 8, 7]
        ratio = 1
        motor2 = motor.Motor(pins, ratio, self.max)
        self.assertTrue(motor2._pins, pins)
        self.assertTrue(motor2._ratio, ratio)
        self.assertTrue(motor2._max_speed, max)

    def test_rotate_cw(self):
        run_time = 5
        start = time.perf_counter()
        asyncio.run(self.motor1.rotate(360, run_time))
        self.assertTrue(abs(time.perf_counter() - start - run_time) < 0.1)

    def test_rotate_acw(self):
        run_time = 5
        start = time.perf_counter()
        asyncio.run(self.motor1.rotate(-360, run_time))
        self.assertTrue(abs(time.perf_counter() - start - run_time) < 0.1)

    def test_motor_max(self):
        run_time = 0
        start = time.perf_counter()
        asyncio.run(self.motor1.rotate(360, run_time))
        self.assertTrue(abs(time.perf_counter() - start - (360/self.max)) < 0.25)

    def test_motor_zero(self):
        run_time = 0
        angle = 0
        start = time.perf_counter()
        asyncio.run(self.motor1.rotate(angle, run_time))
        self.assertTrue(abs(time.perf_counter() - start - run_time) < 0.1)

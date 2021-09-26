import asyncio
import time
from unittest import TestCase
import motor


class Test(TestCase):
    motor = [24, 25, 8, 7]

    def test_rotate_cw(self):
        run_time = 5
        start = time.perf_counter()
        asyncio.run(motor.rotate(self.motor, 365, run_time))
        self.assertTrue(abs(time.perf_counter() - start - run_time) < 0.1)

    def test_rotate_acw(self):
        run_time = 5
        start = time.perf_counter()
        asyncio.run(motor.rotate(self.motor, -365, run_time))
        self.assertTrue(abs(time.perf_counter() - start - run_time) < 0.1)

    def test_motor_max(self):
        run_time = 0
        start = time.perf_counter()
        asyncio.run(motor.rotate(self.motor, 365, run_time))
        self.assertTrue(abs(time.perf_counter() - start - 4) < 0.1)

    def test_motor_zero(self):
        run_time = 0
        angle = 0
        start = time.perf_counter()
        asyncio.run(motor.rotate(self.motor, angle, run_time))
        self.assertTrue(abs(time.perf_counter() - start - run_time) < 0.1)

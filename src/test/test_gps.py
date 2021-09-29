import time

from datetime import datetime as dt, timezone
from unittest import TestCase
import gps


class Test(TestCase):

    def test_get_coordinates(self):
        lat = gps.gps.location.latitude.degrees
        long = gps.gps.location.longitude.degrees
        self.assertLessEqual(lat, 90)
        self.assertGreaterEqual(lat, -90)
        self.assertLessEqual(long, 180)
        self.assertGreaterEqual(long, -180)

    def test_get_time(self):
        t = gps.gps.time.timestamp()
        self.assertTrue((t - dt.now(timezone.utc).timestamp()) * 1000 < 100)

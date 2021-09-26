import os
import unittest
from unittest import TestCase
from skyfield.api import load, wgs84
import main
import scipy.constants as constants


class Test(TestCase):
    location = wgs84.latlon(57, -7)
    all_sats = load.tle_file('../noaa.txt')
    by_name = {sat.name: sat for sat in all_sats}
    noaa_15 = by_name['NOAA 15 [B]']
    noaa_18 = by_name['NOAA 18 [B]']
    noaa_19 = by_name['NOAA 19 [+]']
    noaa_1 = by_name['NOAA 1 [-]']


    def test_get_frequency_valid(self):
        f15_actual = int(137620000)
        f18_actual = int(137912500)
        f19_actual = int(137100000)
        self.assertEqual(main.get_frequency(self.noaa_15), f15_actual)
        self.assertEqual(main.get_frequency(self.noaa_18), f18_actual)
        self.assertEqual(main.get_frequency(self.noaa_19), f19_actual)

    def test_get_frequency_invalid(self):
        with self.assertRaises(ValueError):
            main.get_frequency(self.noaa_1)

    def test_get_doppler_shift(self):
        (_, _, _, _, _, range_rate) = (self.noaa_15 - self.location).at(load.timescale().now()).frame_latlon_and_rates(self.location)
        f15_actual = int(137620000) * range_rate.m_per_s / constants.c
        self.assertAlmostEqual(main.get_doppler_shift(self.noaa_15), f15_actual, 0)

    def test_check_tle_epoch(self):
        self.assertFalse(main.check_tle_epoch('x.txt'))
        self.assertFalse(main.check_tle_epoch('test/noaaoutdated.txt'))
        self.assertTrue(main.check_tle_epoch('noaanotoutdated.txt'))

    def test_update_tle(self):
        main.update_tle('noaatest.txt')
        self.assertTrue(os.path.exists('noaatest.txt'))
        self.assertTrue(main.check_tle_epoch('noaatest.txt'))



if __name__ == '__main__':
    unittest.main()

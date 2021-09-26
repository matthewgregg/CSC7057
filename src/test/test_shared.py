from unittest import TestCase
import shared


class TestRunning(TestCase):

    running = shared.Running

    def set_value(self, value):
        self.running = value

    def test_get_set_value_valid(self):
        val = True
        self.set_value(val)
        self.assertEqual(self.running, val)

    def test_set_value_invalid(self):
        self.assertRaises(TypeError, self.set_value(1))

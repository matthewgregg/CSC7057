import asyncio
import time
import RPi.GPIO as GPIO

GPIO.setmode(GPIO.BCM)

class Motor(object):
    """Motor object to drive motor"""
    # Half step drive sequence
    SEQUENCE = [[1, 0, 0, 0],
                [1, 1, 0, 0],
                [0, 1, 0, 0],
                [0, 1, 1, 0],
                [0, 0, 1, 0],
                [0, 0, 1, 1],
                [0, 0, 0, 1],
                [1, 0, 0, 1]]

    STEPS_PER_REV = 2048
    rotation_seq_count = len(SEQUENCE)

    def __init__(self, pins: list, ratio: float, max_speed: float):
        """Initialise motor object for motor at specified pins

        :param pins: the GPIO pins for the motor
        :param ratio: the gear ratio of the motor
        :param max_speed: the maximum speed of the motor
        """

        for pin in pins:
            GPIO.setup(pin, GPIO.OUT)
            GPIO.output(pin, False)

        self._pins = pins
        self._ratio = ratio
        self._max_speed = max_speed

    async def rotate(self, angle: float, duration: float) -> None:
        """Rotate the motor by an angle over a duration

        :param angle: the angle to rotate by
        :param duration: the duration to rotate the angle over
        :return: None
        """
        angle *= self._ratio
        if duration < 0:
            raise ValueError
        if angle == 0:
            if duration > 0:
                await asyncio.sleep(duration)
            return
        if duration == 0 or angle / duration > self._max_speed:
            duration = abs(angle / self._max_speed)
        start = time.perf_counter()
        sequence_count = 0
        if angle > 0:
            plus_minus = 1
        else:
            plus_minus = -1
        # Times 2 because half-step
        steps = 2 * abs(int(float(angle) / 360 * self.STEPS_PER_REV))
        for i in range(steps):
            for pin in range(4):
                current_pin = self._pins[pin]
                if self.SEQUENCE[sequence_count][pin] != 0:
                    GPIO.output(current_pin, True)
                else:
                    GPIO.output(current_pin, False)
            sequence_count += plus_minus
            # If we reach the end of the sequence start again
            if sequence_count == self.rotation_seq_count:
                sequence_count = 0
            if sequence_count < 0:
                sequence_count = self.rotation_seq_count - 1
            # Wait to match entered duration
            wait = (float(i) / steps * duration) - (time.perf_counter() - start)
            if wait > 0:
                await asyncio.sleep(wait)
        for pin in self._pins:
            GPIO.output(pin, False)
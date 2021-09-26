import asyncio
import time
import RPi.GPIO as GPIO

GPIO.setmode(GPIO.BCM)

class Motor(object):

    def __init__(self, pins, ratio):
        for pin in pins:
            GPIO.setup(pin, GPIO.OUT)
            GPIO.output(pin, False)

        self.pins = pins
        self.ratio = ratio

        self.rotation_seq_count = 8
        # half step drive sequence
        self.seq = [[1, 0, 0, 0],
               [1, 1, 0, 0],
               [0, 1, 0, 0],
               [0, 1, 1, 0],
               [0, 0, 1, 0],
               [0, 0, 1, 1],
               [0, 0, 0, 1],
               [1, 0, 0, 1]]

        self.steps_per_rev = 2048


    async def rotate(self, angle, duration):
        angle *= self.ratio
        if duration < 0:
            raise ValueError
        if angle == 0:
            if duration > 0:
                await duration
            return
        # max speed = 90 deg/sec
        if duration == 0 or angle / duration > 90:
            duration = angle / 90
        start = time.perf_counter()
        sequence_count = 0
        if angle > 0:
            plus_minus = 1
        else:
            plus_minus = -1
        steps = 2 * abs(int(float(angle) / 360 * self.steps_per_rev))  # times 2 because half-step
        for i in range(steps):
            for pin in range(4):
                current_pin = self.pins[pin]
                if self.seq[sequence_count][pin] != 0:
                    GPIO.output(current_pin, True)
                else:
                    GPIO.output(current_pin, False)
            sequence_count += plus_minus
            # If we reach the end of the sequence start again
            if sequence_count == self.rotation_seq_count:
                sequence_count = 0
            if sequence_count < 0:
                sequence_count = self.rotation_seq_count - 1
            # wait to match entered duration
            wait = (float(i) / steps * abs(duration)) - (time.perf_counter() - start)
            if wait > 0:
                await asyncio.sleep(wait)
        for pin in self.pins:
            GPIO.output(pin, False)

rotation_seq_count = 8
# half step drive sequence
seq = [[1, 0, 0, 0],
       [1, 1, 0, 0],
       [0, 1, 0, 0],
       [0, 1, 1, 0],
       [0, 0, 1, 0],
       [0, 0, 1, 1],
       [0, 0, 0, 1],
       [1, 0, 0, 1]]

steps_per_rev = 2048
async def rotate(pins, angle, duration):
    if duration < 0:
        raise ValueError
    if angle == 0:
        if duration > 0:
            await duration
        return
    # max speed = 90 deg/sec
    if duration == 0 or angle / duration > 90:
        duration = angle / 90
    start = time.perf_counter()
    sequence_count = 0
    if angle > 0:
        plus_minus = 1
    else:
        plus_minus = -1
    steps = 2 * abs(int(float(angle) / 360 * steps_per_rev))  # times 2 because half-step
    for i in range(steps):
        for pin in range(4):
            current_pin = pins[pin]
            if seq[sequence_count][pin] != 0:
                GPIO.output(current_pin, True)
            else:
                GPIO.output(current_pin, False)
        sequence_count += plus_minus
        # If we reach the end of the sequence start again
        if sequence_count == rotation_seq_count:
            sequence_count = 0
        if sequence_count < 0:
            sequence_count = rotation_seq_count - 1
        # wait to match entered duration
        wait = (float(i) / steps * abs(duration)) - (time.perf_counter() - start)
        if wait > 0:
            await asyncio.sleep(wait)
    for pin in pins:
        GPIO.output(pin, False)
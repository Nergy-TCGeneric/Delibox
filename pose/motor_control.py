import math
from gpiozero import Motor, DigitalInputDevice
import time
from enum import Enum

# Coded by Tae hyeon, Jung.


class Direction(Enum):
    FORWARD = 1
    BACKWARD = -1

class WheelEncoder:
    _tick_per_second: float = 0
    _encoder_device: DigitalInputDevice
    _direction: Direction = Direction.FORWARD
    _last_timestamp: float

    def __init__(self, pin) -> None:
        self._encoder_device = DigitalInputDevice(pin=pin)
        self._encoder_device.when_activated = self._calculate_time_diff
        self._last_timestamp = time.time()

    def _calculate_time_diff(self) -> None:
        current_time = time.time()
        if self._last_timestamp is not None:
            # This assumes an ideal wheel motion.
            elasped_time = current_time - self._last_timestamp
            self._tick_per_second = 1 / elasped_time
        self._last_timestamp = current_time

    def get_angular_velocity(self) -> float:
        # Since wheel encoder has 20 holes, we calculated this by doing 2 * math.pi / 20.
        return math.pi / 10 * self._tick_per_second * self._direction.value

    def change_direction(self, new_dir: Direction) -> None:
        self._direction = new_dir

class MotorControl:
    # These are all pre-defined GPIO configuration.
    # Change this if pin connection has altered.
    motor = Motor(forward=14, backward=15)
    motor2 = Motor(forward=17, backward=27)
    motor3 = Motor(forward=20, backward=21)
    motor4 = Motor(forward=13, backward=19)

    left_encoder = WheelEncoder(8)
    right_encoder = WheelEncoder(7)

    def front(self, speed=0.3):
        self.motor.forward(speed)
        self.motor2.forward(speed)
        self.motor3.forward(speed)
        self.motor4.forward(speed)

    def stop(self):
        self.motor.stop()
        self.motor2.stop()
        self.motor3.stop(),
        self.motor4.stop(),

    def back(self, speed=0.3):
        self.motor.backward(speed)
        self.motor2.backward(speed)
        self.motor3.backward(speed)
        self.motor4.backward(speed)

    def right(self, speed=0.3):
        self.motor.forward(speed)
        self.motor3.forward(speed)
        self.motor2.backward(speed)
        self.motor4.backward(speed)

    def left(self, speed=0.3):
        self.motor2.forward(speed)
        self.motor4.forward(speed)
        self.motor.backward(speed)
        self.motor3.backward(speed)

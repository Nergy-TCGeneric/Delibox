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
    front_right = Motor(forward=14, backward=15)
    front_left = Motor(forward=27, backward=17)
    rear_left = Motor(forward=20, backward=21)
    rear_right = Motor(forward=16, backward=26)

    left_encoder = WheelEncoder(8)
    right_encoder = WheelEncoder(7)

    def front(self, speed=0.3):
        self.front_right.forward(speed)
        self.front_left.forward(speed)
        self.rear_left.forward(speed)
        self.rear_right.forward(speed)

        self.left_encoder.change_direction(Direction.FORWARD)
        self.right_encoder.change_direction(Direction.FORWARD)

    def stop(self):
        self.front_right.stop()
        self.front_left.stop()
        self.rear_left.stop(),
        self.rear_right.stop(),

    def back(self, speed=0.3):
        self.front_right.backward(speed)
        self.front_left.backward(speed)
        self.rear_left.backward(speed)
        self.rear_right.backward(speed)

        self.left_encoder.change_direction(Direction.BACKWARD)
        self.right_encoder.change_direction(Direction.BACKWARD)

    def right(self, speed=0.3):
        self.front_right.forward(speed)
        self.rear_left.forward(speed)
        self.front_left.backward(speed)
        self.rear_right.backward(speed)

        self.left_encoder.change_direction(Direction.FORWARD)
        self.right_encoder.change_direction(Direction.BACKWARD)

    def left(self, speed=0.3):
        self.front_left.forward(speed)
        self.rear_right.forward(speed)
        self.front_right.backward(speed)
        self.rear_left.backward(speed)

        self.left_encoder.change_direction(Direction.BACKWARD)
        self.right_encoder.change_direction(Direction.FORWARD)

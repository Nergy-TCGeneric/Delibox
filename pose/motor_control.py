from gpiozero import Motor

# Coded by Tae hyeon, Jung.


class MotorControl:
    # These are all pre-defined GPIO configuration.
    # Change this if pin connection has altered.
    motor = Motor(forward="GPIO6", backward="GPIO13")
    motor2 = Motor(forward="GPIO19", backward="GPIO26")
    motor3 = Motor(forward="GPIO15", backward="GPIO14")
    motor4 = Motor(forward="GPIO18", backward="GPIO23")

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

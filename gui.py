#!/bin/python3
# fist_made prototype

import math
import sys
from pose import motor_control
from PyQt5.QtWidgets import (
    QWidget,
    QPushButton,
    QApplication,
    QLabel,
    QGridLayout,
    QHBoxLayout,
    QVBoxLayout,
    QSlider,
)
from PyQt5.QtCore import Qt

from lidar import g2

MIN_SPEED_DIAL = 1
MAX_SPEED_DIAL = 100

resolution_w = 640
resolution_h = 480

button_size = math.floor(
    (resolution_w if resolution_w < resolution_h else resolution_h) / 10
)

port = input("Please enter the port of G2: ")
lidar = g2.G2(port)


class BodyControl(QWidget):
    # motor_ctrl = motor_control.MotorControl()
    speed = 0

    def __init__(self):
        super().__init__()
        self.initUI()

    def initUI(self):
        # layout setting
        v_layout = QVBoxLayout()
        grid_layout = QGridLayout()
        kill_layout = QHBoxLayout()
        kill_layout.addStretch()
        pow_slider_layout = QVBoxLayout()

        power_slider = QSlider(Qt.Orientation.Horizontal, self)
        power_slider.setRange(MIN_SPEED_DIAL, MAX_SPEED_DIAL)

        # grid_button_layout
        button_forward = QPushButton("↑", self)
        button_forward.clicked.connect(self.moveForward)
        grid_layout.addWidget(button_forward, 0, 1)

        button_backword = QPushButton("↓", self)
        button_backword.clicked.connect(self.moveBackward)
        grid_layout.addWidget(button_backword, 2, 1)

        button_turnleft = QPushButton("←", self)
        button_turnleft.clicked.connect(self.turnLeft)
        grid_layout.addWidget(button_turnleft, 1, 0)

        button_turnright = QPushButton("→", self)
        button_turnright.clicked.connect(self.turnRight)
        grid_layout.addWidget(button_turnright, 1, 2)

        button_kill = QPushButton("X", self)
        button_kill.clicked.connect(self.close)
        kill_layout.addWidget(button_kill)

        button_stop = QPushButton("-", self)
        button_stop.clicked.connect(self.stop)
        grid_layout.addWidget(button_stop, 1, 1)
        grid_layout.setAlignment(Qt.AlignmentFlag.AlignCenter)

        # power_slide_layout
        power_slider.valueChanged.connect(self.slider_position)
        self.power_status_label = QLabel("1", self)
        pow_slider_layout.addWidget(power_slider)
        pow_slider_layout.addWidget(self.power_status_label)

        kill_layout.setContentsMargins(
            button_size * 9, button_size, math.floor(button_size / 2), 0
        )
        v_layout.addLayout(kill_layout)
        v_layout.addLayout(pow_slider_layout)
        v_layout.addLayout(grid_layout)
        self.setLayout(v_layout)
        self.setGeometry(200, 150, resolution_w, resolution_h)
        self.setWindowTitle("Prototype")
        self.show()

        # When everything is done, enable the G2
        lidar.enable()

    # buttons_actions_funcs
    def moveForward(self):
        self.motor_ctrl.front(self.speed)

    def moveBackward(self):
        self.motor_ctrl.back(self.speed)

    def turnLeft(self):
        self.motor_ctrl.left(self.speed)

    def turnRight(self):
        self.motor_ctrl.right(self.speed)

    def stop(self):
        self.motor_ctrl.stop()

    def close(self):
        lidar.disable()
        QApplication.instance().quit()

    def slider_position(self, p):
        self.power_status_label.setText(str(p))
        self.speed = float(p / MAX_SPEED_DIAL)


def main():
    app = QApplication(sys.argv)
    app.processEvents()
    BC = BodyControl()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()

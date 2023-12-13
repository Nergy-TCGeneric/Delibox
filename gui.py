#!/bin/python3
# fist_made prototype

import math
import sys
from pose import motor_control
from mapper import mapper

from PyQt5.QtWidgets import (
    QWidget,
    QPushButton,
    QApplication,
    QLabel,
    QGridLayout,
    QHBoxLayout,
    QVBoxLayout,
    QSlider,
    QScrollArea,
)
from PyQt5.QtCore import QObject, Qt, QThread, pyqtSignal, QTimer
from PyQt5.QtGui import QPalette, QColor, QImage, QPixmap, QPainter

from lidar import g2

MM_TO_PX_RESOLUTION = 20
MIN_SPEED_DIAL = 1
MAX_SPEED_DIAL = 100

resolution_w = 640
resolution_h = 480

button_size = math.floor(
    (resolution_w if resolution_w < resolution_h else resolution_h) / 10
)


class GridWorker(QThread):
    finished = pyqtSignal()
    _lidar: g2.G2
    _local_mapper: mapper.Submapper
    _global_mapper = mapper.GlobalMapper

    def __init__(self, parent: QObject, port: str) -> None:
        super().__init__(parent)
        self._lidar = g2.G2(port)
        self._local_mapper = mapper.Submapper(MM_TO_PX_RESOLUTION)
        self._global_mapper = mapper.GlobalMapper((50, 50))
        self._lidar.enable()

    def run(self) -> None:
        scanned = self._lidar.read_data()
        mapped = self._local_mapper.lidar_to_submap(scanned)
        self._global_mapper.update(mapped)
        self.finished.emit(self._global_mapper._occupancy_grid)

    def update_observer_pos(self, pos: mapper.Point) -> None:
        self._global_mapper.update_observer_pos(pos)


class BodyControl(QWidget):
    motor_ctrl = motor_control.MotorControl()
    speed = 0
    _grid_worker: GridWorker
    _encoder_timer: QTimer
    _is_moving: bool = False

    # Variable for odometry. x, y and orientation respectively.
    _state = [
        0.0,
        0.0,
        0.0,
    ]

    def __init__(self):
        super().__init__()
        self.initUI()

        self._encoder_timer = QTimer(self)
        self._encoder_timer.setInterval(100)  # Check it for 100ms
        self._grid_worker = GridWorker(self, "/dev/ttyUSB0")
        self._grid_worker.finished.connect(self.update_image)

    def initUI(self):
        # layout setting
        v_layout = QVBoxLayout()
        grid_layout = QGridLayout()
        kill_layout = QHBoxLayout()
        kill_layout.addStretch()
        pow_slider_layout = QVBoxLayout()
        self.image_scroll_layout = QScrollArea()

        power_slider = QSlider(Qt.Orientation.Horizontal, self)
        power_slider.setRange(MIN_SPEED_DIAL, MAX_SPEED_DIAL)

        self.imageLabel = QLabel()
        self.imageLabel.setScaledContents(True)

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

    # buttons_actions_funcs
    def moveForward(self):
        self.motor_ctrl.front(self.speed)
        if not self._encoder_timer.isActive():
            self._encoder_timer.start()

    def moveBackward(self):
        self.motor_ctrl.back(self.speed)
        if not self._encoder_timer.isActive():
            self._encoder_timer.start()

    def turnLeft(self):
        self.motor_ctrl.left(self.speed)
        if not self._encoder_timer.isActive():
            self._encoder_timer.start()

    def turnRight(self):
        self.motor_ctrl.right(self.speed)
        if not self._encoder_timer.isActive():
            self._encoder_timer.start()

    def update_image(self, map: mapper.Map):
        # TODO: Won't work, this requires numpy array.
        # Also see this : https://stackoverflow.com/questions/48639185/pyqt5-qimage-from-numpy-array
        image = QImage(
            map.content,
            map.content.shape[1],
            map.content.shape[0],
            QImage.Format.Format_Grayscale8,
        )
        self.imageLabel.setPixmap(QPixmap.fromImage(image))
        self.image_scroll_layout.setWidget(self.imageLabel)

    def update_observer_position(self):
        lv = self.motor_ctrl.left_encoder.get_angular_velocity() * 33
        rv = self.motor_ctrl.right_encoder.get_angular_velocity() * 33
        orientation = (rv - lv) / 115
        mv = (lv + rv) / 2

        dx = math.cos(orientation) * mv
        dy = math.sin(orientation) * mv
        self._state[0] = self._state[0] + dx
        self._state[1] = self._state[1] + dx
        self._state[2] = self._state[2] + orientation

        # Diving it by resolution to yield a correct point
        adjusted_x = int(self._state[0]) // 20
        adjusted_y = int(self._state[1]) // 20
        self._grid_worker.update_observer_pos(mapper.Point(adjusted_x, adjusted_y))

    def stop(self):
        self.motor_ctrl.stop()
        if self._encoder_timer.isActive():
            self._encoder_timer.stop()

    def close(self):
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

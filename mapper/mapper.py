from collections import deque
from mapper import bresenham
from lidar import g2
from typing import List
from typing import Final
from dataclasses import dataclass
import math


@dataclass
class AdjustedPoint:
    x: int
    y: int

    def is_zero_point(self) -> bool:
        return self.x == BIAS and self.y == BIAS

    def is_in_range(self) -> bool:
        return (self.x >= 0 and self.x < MAP_SIZE) and (
            self.y >= 0 and self.y < MAP_SIZE
        )


MAP_SIZE: Final[int] = 5000
BIAS: Final[int] = MAP_SIZE // 2

# State constants.
OCCUPIED: Final[int] = 255
UNCERTAIN: Final[int] = 128
FREE: Final[int] = 0


class Mapper:
    # 2D array for storing maps
    occupancy_grid: list[list[int]]

    def __init__(self):
        # https://docs.python.org/3/faq/programming.html#how-do-i-create-a-multidimensional-list
        # Careful! This map's size is roughly 252MB, which is really HUGE for some embedded devices.
        self.occupancy_grid = [None] * MAP_SIZE
        for i in range(0, MAP_SIZE):
            self.occupancy_grid[i] = [UNCERTAIN] * MAP_SIZE

    def lidar_to_grid(self, points: List[g2.LaserScanPoint]):
        origin = AdjustedPoint(BIAS, BIAS)
        adjusted = self._get_adjusted_points(points)
        self._draw_outer_lines(adjusted)
        self._flood_fill(origin)
        self._emphasize_walls(adjusted)

    def _get_adjusted_points(
        self, points: List[g2.LaserScanPoint]
    ) -> list[AdjustedPoint]:
        clamped: list[AdjustedPoint] = []
        for point in points:
            # This requires towards-zero rounding, as using ceil() and floor() might slightly offset results.
            # As rasterized points can be negative, we need to add bias to make sure
            # coordinates are greater or equal to 0.
            x: int = int(point.distance * math.cos(point.radian)) + BIAS
            y: int = int(point.distance * math.sin(point.radian)) + BIAS

            # We need to retain informations of points with zero distances.
            clamped_x: int = self._clamp(x, 0, MAP_SIZE - 1)
            clamped_y: int = self._clamp(y, 0, MAP_SIZE - 1)
            p = AdjustedPoint(clamped_x, clamped_y)

            clamped.append(p)

        return clamped

    def _draw_outer_lines(self, adjusted: list[AdjustedPoint]):
        points = deque()

        # TODO: Use numpy instead, because we need a vectorization to speed up this process.
        for p in adjusted:
            if not p.is_zero_point():
                points.append(p)

            # When it's possible to draw a outer line, do it.
            if len(points) == 2:
                self._draw_line(points[0], points[1])
                points.popleft()

        # Final touch, as this segment is disconnected at first.
        self._draw_line(points[0], adjusted[0])

    def _emphasize_walls(self, adjusted: list[AdjustedPoint]):
        # Thicken the walls to 2px to make it more visible.
        for point in adjusted:
            self.occupancy_grid[point.y][point.x] = OCCUPIED

            if point.y + 1 < MAP_SIZE:
                self.occupancy_grid[point.y + 1][point.x] = OCCUPIED
            if point.x + 1 < MAP_SIZE:
                self.occupancy_grid[point.y][point.x + 1] = OCCUPIED
            if point.y + 1 < MAP_SIZE and point.x + 1 < MAP_SIZE:
                self.occupancy_grid[point.y + 1][point.x + 1] = OCCUPIED

    def _draw_line(self, p1: AdjustedPoint, p2: AdjustedPoint):
        line = bresenham.bresenham(p1.x, p2.x, p1.y, p2.y)
        for l in line:
            self.occupancy_grid[l[1]][l[0]] = FREE

    def _flood_fill(self, start: AdjustedPoint):
        queue: list[AdjustedPoint] = []
        queue.append(start)

        while len(queue) > 0:
            current = queue.pop()

            # Search for four directions, respectively.
            self._fill_up_free_cells(queue, AdjustedPoint(current.x, current.y + 1))
            self._fill_up_free_cells(queue, AdjustedPoint(current.x, current.y - 1))
            self._fill_up_free_cells(queue, AdjustedPoint(current.x + 1, current.y))
            self._fill_up_free_cells(queue, AdjustedPoint(current.x - 1, current.y))

    def _fill_up_free_cells(self, queue: list[AdjustedPoint], p: AdjustedPoint):
        if not p.is_in_range():
            return

        state = self.occupancy_grid[p.y][p.x]
        if state == UNCERTAIN:
            self.occupancy_grid[p.y][p.x] = FREE
            queue.append(p)

    def _clamp(self, a: int, min_n: int, max_n: int) -> int:
        return min(max(a, min_n), max_n)

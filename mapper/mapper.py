from collections import deque
from operator import attrgetter
from mapper import bresenham
from lidar import g2
from typing import List
from typing import Final
from dataclasses import dataclass
import math


@dataclass
class Point:
    x: int
    y: int

# State constants.
OCCUPIED: Final[int] = 255
UNCERTAIN: Final[int] = 128
FREE: Final[int] = 0


class Mapper:
    # 2D array for storing maps
    occupancy_grid: list[list[int]]
    resolution: int
    _dimension: tuple[int, int]

    def __init__(self, resolution) -> None:
        if resolution < 1:
            raise ValueError("Resolution cannot be less than 1.")
        self.resolution = resolution

    def lidar_to_grid(self, points: List[g2.LaserScanPoint]) -> None:
        rasterized = self._rasterize_points(points)
        self._setup_grid(rasterized)
        adjusted = self._adjust_points(rasterized)

        center = self._get_center_point()
        self._draw_outer_lines(adjusted)
        self._flood_fill(Point(center[0], center[1]))
        self._emphasize_walls(adjusted)

    def _setup_grid(self, points: List[Point]) -> None:
        # https://stackoverflow.com/a/6085482
        # An adaptive approach to scale occupancy grid.
        # We add some immediates behind to 'pad' the map.
        min_x = min(points, key=attrgetter("x")).x - 1
        min_y = min(points, key=attrgetter("y")).y - 1
        max_x = max(points, key=attrgetter("x")).x + 1
        max_y = max(points, key=attrgetter("y")).y + 1

        x_width = (max_x - min_x) // self.resolution
        y_width = (max_y - min_y) // self.resolution

        # By doing this way, one should access grid like grid[y][x].
        self.occupancy_grid = [
            [UNCERTAIN for i in range(x_width)] for j in range(y_width)
        ]
        self._dimension = (x_width, y_width)

    def _adjust_points(self, points: List[Point]) -> List[Point]:
        x_center, y_center = self._get_center_point()

        adjusted = []
        for p in points:
            a_x = self._clamp(p.x + x_center, 0, x_center * 2 - 1)
            a_y = self._clamp(p.y + y_center, 0, y_center * 2 - 1)
            adjusted.append(Point(a_x, a_y))

        return adjusted

    def _get_center_point(self) -> tuple[int, int]:
        if self._dimension is None:
            return (0, 0)

        x_width = self._dimension[0]
        y_width = self._dimension[1]
        return (x_width // 2, y_width // 2)

    def _rasterize_points(self, points: List[g2.LaserScanPoint]) -> list[Point]:
        clamped: list[Point] = []
        for point in points:
            # This requires towards-zero rounding, as using ceil() and floor() might slightly offset results.
            # As rasterized points can be negative, we need to add bias to make sure
            # coordinates are greater or equal to 0.
            x: int = int(point.distance * math.cos(point.radian)) // self.resolution
            y: int = int(point.distance * math.sin(point.radian)) // self.resolution
            p = Point(x, y)

            clamped.append(p)

        return clamped

    def _draw_outer_lines(self, adjusted: list[Point]):
        points = deque()

        for p in adjusted:
            if not self._is_zero_point(p):
                points.append(p)

            # When it's possible to draw a outer line, do it.
            if len(points) == 2:
                self._draw_line(points[0], points[1])
                points.popleft()

        # Final touch, as this segment is disconnected at first.
        self._draw_line(points[0], adjusted[0])

    def _is_zero_point(self, point: Point) -> bool:
        center = self._get_center_point()
        return point.x == center[0] and point.y == center[1]

    def _emphasize_walls(self, adjusted: list[Point]):
        # Thicken the walls to 2px to make it more visible.
        x_width, y_width = self._dimension
        for point in adjusted:
            self.occupancy_grid[point.y][point.x] = OCCUPIED

            if point.y + 1 < y_width:
                self.occupancy_grid[point.y + 1][point.x] = OCCUPIED
            if point.x + 1 < x_width:
                self.occupancy_grid[point.y][point.x + 1] = OCCUPIED
            if point.y + 1 < y_width and point.x + 1 < x_width:
                self.occupancy_grid[point.y + 1][point.x + 1] = OCCUPIED

    def _draw_line(self, p1: Point, p2: Point):
        line = bresenham.bresenham(p1.x, p2.x, p1.y, p2.y)
        for l in line:
            self.occupancy_grid[l[1]][l[0]] = FREE

    def _flood_fill(self, start: Point):
        queue: list[Point] = []
        queue.append(start)

        while len(queue) > 0:
            current = queue.pop()

            # Search for four directions, respectively.
            self._fill_up_free_cells(queue, Point(current.x, current.y + 1))
            self._fill_up_free_cells(queue, Point(current.x, current.y - 1))
            self._fill_up_free_cells(queue, Point(current.x + 1, current.y))
            self._fill_up_free_cells(queue, Point(current.x - 1, current.y))

    def _fill_up_free_cells(self, queue: list[Point], p: Point):
        if not self._is_point_in_grid(p):
            return

        state = self.occupancy_grid[p.y][p.x]
        if state == UNCERTAIN:
            self.occupancy_grid[p.y][p.x] = FREE
            queue.append(p)

    def _is_point_in_grid(self, point: Point) -> bool:
        x_width, y_width = self._dimension
        return (point.x >= 0 and point.x < x_width) and (
            point.y >= 0 and point.y < y_width
        )

    def _clamp(self, a: int, min_n: int, max_n: int) -> int:
        return min(max(a, min_n), max_n)

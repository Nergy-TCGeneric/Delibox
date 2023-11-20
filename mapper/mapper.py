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
        return self.x == 0 and self.y == 0

    def is_in_range(self) -> bool:
        return (self.x > 0 and self.x < MAP_SIZE) and (self.y > 0 and self.y < MAP_SIZE)

MAP_SIZE: Final[int] = 5000
BIAS: Final[int] = MAP_SIZE // 2

# State constants.
OCCUPIED: Final[float] = 1
UNCERTAIN: Final[float] = 0.5
FREE: Final[float] = 0

class Mapper:
    # 2D array for storing maps
    occupancy_grid: List[List[float]]

    def __init__(self):
        # https://docs.python.org/3/faq/programming.html#how-do-i-create-a-multidimensional-list
        # Careful! This map's size is roughly 252MB, which is really HUGE for some embedded devices.
        self.occupancy_grid = [None] * MAP_SIZE
        for i in range(0, MAP_SIZE):
            self.occupancy_grid[i] = [UNCERTAIN] * MAP_SIZE

    def lidar_to_grid(self, points: List[g2.LaserScanPoint]):
        adjusted = self._get_adjusted_points(points)
        self._draw_free_spaces(adjusted)
        self._draw_walls(adjusted)

    def _get_adjusted_points(self, points: List[g2.LaserScanPoint]) -> list[AdjustedPoint]:
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

    def _draw_free_spaces(self, adjusted: list[AdjustedPoint]):
        origin = AdjustedPoint(BIAS, BIAS)
        # TODO: Use numpy instead, because we need a vectorization to speed up this process.
        for i in range(len(adjusted) - 1):
            p1, p2 = adjusted[i], adjusted[i+1]

            # Do not proceed if one of point is invalid, i.e, point with zero distance.
            if p1.is_zero_point() or p2.is_zero_point():
                continue

            # Draw a triangle.
            self._draw_line(origin, p1)
            self._draw_line(origin, p2)
            self._draw_line(p1, p2)

            # Flood fill.
            centroid_x = (p1.x + p2.x + BIAS) // 3
            centroid_y = (p1.y + p2.y + BIAS) // 3
            self._flood_fill(AdjustedPoint(centroid_x, centroid_y))

    def _draw_walls(self, adjusted: list[AdjustedPoint]):
        for point in adjusted:
           self.occupancy_grid[point.y][point.x] = OCCUPIED

    def _draw_line(self, p1: AdjustedPoint, p2: AdjustedPoint):
        line = bresenham.bresenham(p1.x, p2.x, p1.y, p2.y)
        for l in line :
            self.occupancy_grid[l[1]][l[0]] = FREE

    def _flood_fill(self, start: AdjustedPoint):
        queue: list[AdjustedPoint] = []
        queue.append(start)

        while len(queue) > 0:
            current = queue.pop()

            # Search for four directions, respectively.
            self._fill_up_free_cells(queue, AdjustedPoint(current.y, current.x + 1))
            self._fill_up_free_cells(queue, AdjustedPoint(current.y, current.x - 1))
            self._fill_up_free_cells(queue, AdjustedPoint(current.y + 1, current.x))
            self._fill_up_free_cells(queue, AdjustedPoint(current.y - 1, current.x))
    
    def _fill_up_free_cells(self, queue: list[AdjustedPoint], p: AdjustedPoint):
        if not p.is_in_range():
            return

        state = self.occupancy_grid[p.y][p.x]
        if state == UNCERTAIN:
            self.occupancy_grid[p.y][p.x] = FREE
            queue.append(p)

    def _clamp(self, a: int, min_n: int, max_n: int) -> int:
        return min(max(a, min_n), max_n)
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

MAP_MAX_RANGE: Final[int] = 5000
MAP_SIZE: Final[int] = MAP_MAX_RANGE - g2.MIN_RANGE
BIAS: Final[int] = (MAP_SIZE- g2.MIN_RANGE) // 2

class Mapper:
    # 2D array for storing maps
    occupancy_grid: List[List[int]]

    def __init__(self):
        # https://docs.python.org/3/faq/programming.html#how-do-i-create-a-multidimensional-list
        # Careful! This map's size is roughly 252MB, which is really HUGE for some embedded devices.
        self.occupancy_grid = [None] * MAP_SIZE
        for i in range(0, MAP_SIZE):
            self.occupancy_grid[i] = [0.5] * MAP_SIZE

    def lidar_to_grid(self, points: List[g2.LaserScanPoint]):
        adjusted = self._get_adjusted_points(points)
        self._draw_free_spaces(adjusted)

    def _get_adjusted_points(self, points: List[g2.LaserScanPoint]) -> list[AdjustedPoint]:
        clamped: list[AdjustedPoint] = []
        for point in points:
            # This requires towards-zero rounding, as using ceil() and floor() might slightly offset results.
            # As rasterized points can be negative, we need to add bias to make sure
            # coordinates are greater or equal to 0.
            x: int = int(point.distance * math.cos(point.angle)) + BIAS
            y: int = int(point.distance * math.sin(point.angle)) + BIAS

            clamped_x: int = self._clamp(x, g2.MIN_RANGE, MAP_SIZE - 1)
            clamped_y: int = self._clamp(y, g2.MIN_RANGE, MAP_SIZE - 1)
            p = AdjustedPoint(clamped_x, clamped_y)

            clamped.append(p)

        return clamped

    def _draw_free_spaces(self, adjusted: list[AdjustedPoint]):
        # TODO: Use numpy instead, because we need a vectorization to speed up this process.
        for point in adjusted:
            # As we translated points from (0, 0) to (BIAS, BIAS), we need to set the origin point as (BIAS, BIAS).
            rasterized = bresenham.bresenham(BIAS, point.x, BIAS, point.y)
            for p in rasterized:
                self.occupancy_grid[p[1]][p[0]] = 0

    def _clamp(self, a: int, min_n: int, max_n: int) -> int:
        return min(max(a, min_n), max_n)
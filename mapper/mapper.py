from mapper import bresenham
from lidar import g2
from typing import List
from typing import Final
import math

MAP_SIZE: Final[int] = 16000 - 120
BIAS: Final[int] = MAP_SIZE // 2 

class Mapper:
    # 2D array for storing maps
    occupancy_grid: List[List[int]]

    def __init__(self):
        # https://docs.python.org/3/faq/programming.html#how-do-i-create-a-multidimensional-list
        # Careful! This map's size is roughly 252MB, which is really HUGE for some embedded devices.
        self.occupancy_grid = [None] * MAP_SIZE
        for i in range(0, MAP_SIZE):
            self.occupancy_grid[i] = [0] * 15880

    def lidar_to_grid(self, points: List[g2.LaserScanPoint]):
        # This assumes the origin point is (0, 0) at this point.
        # TODO: Use numpy instead, because we need a vectorization to speed up this process.
        for point in points:
            x: int = math.ceil(point.distance * math.cos(point.angle))
            y: int = math.ceil(point.distance * math.sin(point.angle))
            rasterized = bresenham.bresenham(0, x, 0, y)

            for p in rasterized:
                # As rasterized points can be negative, we need to add bias to make sure
                # coordinates are greater or equal to 0. 
                grid_x = p[0] + BIAS
                grid_y = p[1] + BIAS
                self.occupancy_grid[grid_y][grid_x] = 1

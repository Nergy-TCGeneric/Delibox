from collections import deque
from operator import attrgetter
from mapper import bresenham
from lidar import g2
from typing import List
from typing import Final
from dataclasses import dataclass
from measurement import timer
import math


@dataclass
class Point:
    x: int
    y: int


@dataclass
class Map:
    dimension: tuple[int, int]
    content: list[list[int]]

    def __init__(self, dimension: tuple[int, int]) -> None:
        self.dimension = dimension
        x_width, y_width = dimension

        # By doing this way, one should access grid like grid[y][x].
        self.content = [[UNCERTAIN for _ in range(x_width)] for _ in range(y_width)]

    def get_center_point(self) -> tuple[int, int]:
        if self.dimension is None:
            return (0, 0)

        x_width = self.dimension[0]
        y_width = self.dimension[1]
        return (x_width // 2, y_width // 2)

    def is_center_point(self, p: Point) -> bool:
        center = self.get_center_point()
        return p.x == center[0] and p.y == center[1]

    def is_inside(self, p: Point) -> bool:
        x_width, y_width = self.dimension
        return (p.x >= 0 and p.x < x_width) and (p.y >= 0 and p.y < y_width)


# State constants.
OCCUPIED: Final[int] = 255
UNCERTAIN: Final[int] = 128
FREE: Final[int] = 0


class Submapper:
    resolution: int

    def __init__(self, resolution) -> None:
        if resolution < 1:
            raise ValueError("Resolution cannot be less than 1.")
        self.resolution = resolution

    @timer.measure_time_in_ns
    def lidar_to_submap(self, points: List[g2.LaserScanPoint]) -> Map:
        rasterized = self._rasterize_points(points)
        submap = self._setup_submap(rasterized)
        adjusted = self._adjust_points(submap, rasterized)

        center = submap.get_center_point()
        self._draw_outer_lines(submap, adjusted)
        self._flood_fill(submap, Point(center[0], center[1]))
        self._emphasize_walls(submap, adjusted)

        return submap

    def _setup_submap(self, points: List[Point]) -> Map:
        # https://stackoverflow.com/a/6085482
        # An adaptive approach to scale occupancy grid.
        # We add some immediates behind to 'pad' the map.
        min_x = min(points, key=attrgetter("x")).x - 1
        min_y = min(points, key=attrgetter("y")).y - 1
        max_x = max(points, key=attrgetter("x")).x + 1
        max_y = max(points, key=attrgetter("y")).y + 1

        x_width = max_x - min_x
        y_width = max_y - min_y

        return Map((x_width, y_width))

    def _adjust_points(self, map: Map, points: List[Point]) -> List[Point]:
        x_center, y_center = map.get_center_point()

        adjusted = []
        for p in points:
            a_x = self._clamp(p.x + x_center, 0, x_center * 2 - 1)
            a_y = self._clamp(p.y + y_center, 0, y_center * 2 - 1)
            adjusted.append(Point(a_x, a_y))

        return adjusted

    @timer.measure_time_in_ns
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

    @timer.measure_time_in_ns
    def _draw_outer_lines(self, map: Map, adjusted: list[Point]):
        points = deque()

        for p in adjusted:
            if not map.is_center_point(p):
                points.append(p)

            # When it's possible to draw a outer line, do it.
            if len(points) == 2:
                self._draw_line(map, points[0], points[1])
                points.popleft()

        # Final touch, as this segment is disconnected at first.
        self._draw_line(map, points[0], adjusted[0])

    def _emphasize_walls(self, map: Map, adjusted: list[Point]):
        # Thicken the walls to 2px to make it more visible.
        x_width, y_width = map.dimension
        for point in adjusted:
            map.content[point.y][point.x] = OCCUPIED

            if point.y + 1 < y_width:
                map.content[point.y + 1][point.x] = OCCUPIED
            if point.x + 1 < x_width:
                map.content[point.y][point.x + 1] = OCCUPIED
            if point.y + 1 < y_width and point.x + 1 < x_width:
                map.content[point.y + 1][point.x + 1] = OCCUPIED

    def _draw_line(self, map: Map, p1: Point, p2: Point):
        line = bresenham.bresenham(p1.x, p2.x, p1.y, p2.y)
        for l in line:
            map.content[l[1]][l[0]] = FREE

    @timer.measure_time_in_ns
    def _flood_fill(self, map: Map, start: Point):
        queue: list[Point] = []
        queue.append(start)

        while len(queue) > 0:
            current = queue.pop()

            # Search for four directions, respectively.
            self._fill_up_free_cells(map, queue, Point(current.x, current.y + 1))
            self._fill_up_free_cells(map, queue, Point(current.x, current.y - 1))
            self._fill_up_free_cells(map, queue, Point(current.x + 1, current.y))
            self._fill_up_free_cells(map, queue, Point(current.x - 1, current.y))

    def _fill_up_free_cells(self, map: Map, queue: list[Point], p: Point):
        if not map.is_inside(p):
            return

        state = map.content[p.y][p.x]
        if state == UNCERTAIN:
            map.content[p.y][p.x] = FREE
            queue.append(p)

    def _clamp(self, a: int, min_n: int, max_n: int) -> int:
        return min(max(a, min_n), max_n)


class GlobalMapper:
    _occupancy_grid: Map
    observer_pos: Point

    def __init__(self, initial_dimension: tuple[int, int]) -> None:
        self._occupancy_grid = Map(initial_dimension)
        # We set observer's position to (0, 0), the center of the grid.
        self.observer_pos = Point(0, 0)

    def update(self, submap: Map) -> None:
        # TODO: Assuming the submap is in bounds. Change it later
        x_width, y_height = submap.dimension
        grid_x_width, grid_y_height = self._occupancy_grid.dimension

        for y in range(y_height):
            for x in range(x_width):
                g_y = grid_y_height // 2 + self.observer_pos.y + (y - y_height // 2)
                g_x = grid_x_width // 2 + self.observer_pos.x + (x - x_width // 2)

                # Only update the obstacle / free space.
                if submap.content[y][x] == FREE or submap.content[y][x] == OCCUPIED:
                    self._occupancy_grid.content[g_y][g_x] = submap.content[y][x]

    def update_observer_pos(self, new_pos: Point) -> None:
        x_width, y_height = self._occupancy_grid.dimension
        if (
            new_pos.x < -(x_width // 2) + 1
            or new_pos.x > (x_width // 2) - 1
            or new_pos.y < -(y_height // 2) + 1
            or new_pos.y > (y_height // 2) + 1
        ):
            return

        self.observer_pos = new_pos

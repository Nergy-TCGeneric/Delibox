from cgi import print_form
from collections import deque
from operator import attrgetter
from mapper import bresenham
from lidar import g2
from typing import List
from typing import Final
from dataclasses import dataclass
from measurement import timer
import math
import numpy as np


@dataclass
class Point:
    x: int
    y: int

    def __iter__(self):
        yield self.x
        yield self.y


@dataclass
class Map:
    dimension: "tuple[int, int]"
    content: np.ndarray

    def __init__(self, dimension: "tuple[int, int]") -> None:
        self.dimension = dimension
        x_width, y_width = dimension

        # By doing this way, one should access grid like grid[y][x].
        self.content = np.ones((y_width, x_width)) * UNCERTAIN

    def get_center_point(self) -> "tuple[int, int]":
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

    def _rasterize_points(self, points: List[g2.LaserScanPoint]) -> "list[Point]":
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

    def _draw_outer_lines(self, map: Map, adjusted: "list[Point]"):
        points = deque()

        for p in adjusted:
            if not map.is_center_point(p):
                points.append(p)

            # When it's possible to draw a outer line, do it.
            if len(points) == 2:
                self._draw_line(map, points[0], points[1])
                points.popleft()

        # Final touch, as this segment is disconnected at first.
        if len(points) > 0:
            self._draw_line(map, points[0], adjusted[0])

    def _emphasize_walls(self, map: Map, adjusted: "list[Point]"):
        # Thicken the walls to 2px to make it more visible.
        x_width, y_width = map.dimension
        for point in adjusted:
            map.content[point.y, point.x] = OCCUPIED

            if point.y + 1 < y_width:
                map.content[point.y + 1, point.x] = OCCUPIED
            if point.x + 1 < x_width:
                map.content[point.y, point.x + 1] = OCCUPIED
            if point.y + 1 < y_width and point.x + 1 < x_width:
                map.content[point.y + 1, point.x + 1] = OCCUPIED

    def _draw_line(self, map: Map, p1: Point, p2: Point):
        line = bresenham.bresenham(p1.x, p2.x, p1.y, p2.y)
        for l in line:
            map.content[l[1], l[0]] = FREE

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

    def _fill_up_free_cells(self, map: Map, queue: "list[Point]", p: Point):
        if not map.is_inside(p):
            return

        state = map.content[p.y, p.x]
        if state == UNCERTAIN:
            map.content[p.y, p.x] = FREE
            queue.append(p)

    def _clamp(self, a: int, min_n: int, max_n: int) -> int:
        return min(max(a, min_n), max_n)


class GlobalMapper:
    _occupancy_grid: Map
    observer_pos: Point

    def __init__(self, initial_dimension: "tuple[int, int]") -> None:
        self._occupancy_grid = Map(initial_dimension)
        # We set observer's position to (0, 0), the center of the grid.
        self.observer_pos = Point(0, 0)

    def update(self, submap: Map) -> None:
        resized = self._calculate_resized_shape(submap)
        offset = self._get_offset(resized)
        if offset[0] > 0 or offset[1] > 0:
            self._resize_grid(resized)
        self._update_occupancy_grid(submap, offset)

    def update_observer_pos(self, new_pos: Point) -> None:
        self.observer_pos = new_pos

    def _calculate_resized_shape(self, submap: Map) -> "tuple[int, int]":
        # We need to convert these values into float, so that precise calculation is ensured
        submap_width, submap_height = map(float, submap.dimension)
        grid_width, grid_height = self._occupancy_grid.dimension
        obs_x, obs_y = self.observer_pos

        # x and y starts from center of the grid. (Before resizing)
        drawing_x, drawing_y = obs_x + grid_width / 2, obs_y + grid_height / 2

        # We know the decimals always one of these: 0.0 or 0.5.
        # and the python's round() doesn't always work as we thought,
        # so we need to manually round-toward-infinity by ourselves.
        min_x = int(drawing_x - submap_width / 2 - 0.5)
        max_x = int(drawing_x + submap_width / 2 + 0.5)
        min_y = int(drawing_y - submap_height / 2 - 0.5)
        max_y = int(drawing_y + submap_height / 2 + 0.5)

        grid_min_x = min(0, min_x)
        grid_max_x = max(grid_width, max_x)
        grid_min_y = min(0, min_y)
        grid_max_y = max(grid_height, max_y)

        new_grid_width = grid_max_x - grid_min_x
        new_grid_height = grid_max_y - grid_min_y

        return (new_grid_width, new_grid_height)

    def _get_offset(self, new_dim: "tuple[int, int]") -> "tuple[int, int]":
        width, height = self._occupancy_grid.dimension
        return (new_dim[0] - width, new_dim[1] - height)

    def _resize_grid(self, new_dim: "tuple[int, int]") -> None:
        new_grid = Map(new_dim)
        width, height = self._occupancy_grid.dimension
        offset = self._get_offset(new_dim)

        for y in range(height):
            for x in range(width):
                new_x = x + offset[0]
                new_y = y + offset[1]
                new_grid.content[new_y, new_x] = self._occupancy_grid.content[y, x]

        self._occupancy_grid = new_grid

    def _update_occupancy_grid(self, submap: Map, offset: "tuple[int, int]") -> None:
        submap_width, submap_height = submap.dimension
        grid_width, grid_height = self._occupancy_grid.dimension

        # We need to subtract the offset, because it's one from before resizing
        # TODO: Why should we need to do these thing? I don't get it.
        grid_x = int((grid_width - offset[0]) / 2) + self.observer_pos.x
        grid_y = int((grid_height - offset[1]) / 2) + self.observer_pos.y

        for y in range(submap_height):
            for x in range(submap_width):
                submap_x = x - int(submap_width / 2)
                submap_y = y - int(submap_height / 2)
                g_y = grid_y + submap_y
                g_x = grid_x + submap_x

                # The g_y and g_x must be greater or equal to 0.
                assert g_y >= 0
                assert g_x >= 0
                # Only update the obstacle / free space.
                if submap.content[y, x] == FREE or submap.content[y, x] == OCCUPIED:
                    self._occupancy_grid.content[g_y, g_x] = submap.content[y, x]

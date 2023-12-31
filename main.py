from lidar import g2
from mapper import mapper
from typing import List
from mapper.mapper import Point
from measurement import timer
from PIL import Image

import matplotlib.pyplot as plt


def visualize_scan_points(scanned_data: List[g2.LaserScanPoint]):
    fig = plt.figure()
    ax = fig.add_subplot(projection="polar")

    dist = []
    radians = []

    for data in scanned_data:
        dist.append(data.distance)
        radians.append(data.radian)

    ax.scatter(radians, dist, s=0.1)
    plt.show()

def visualize_occupancy_grid(grid: List[List[int]]):
    plt.imshow(grid, cmap="binary", vmin=0, vmax=255)
    plt.draw()
    plt.pause(0.16)


def serialize(grid: list[list[int]]):
    with open("map.bin", "wb") as f:
        for row in grid:
            f.write(bytes(row))


def create_grayscale_bitmap(grid: list[list[int]]):
    height = len(grid)
    width = len(grid[0])
    img = Image.new("L", (width, height))

    for y in range(height):
        for x in range(width):
            img.putpixel((x, y), grid[y][x])

    img.save("grayscale_bitmap.bmp")


port = input("Enter port: ")
g2_lidar = g2.G2(port)

submapper = mapper.Submapper(18)
global_mapper = mapper.GlobalMapper((250, 250))

for i in range(0, 5):
    scanned_data = g2_lidar.read_data_once(10)
    submap = submapper.lidar_to_submap(scanned_data)
    global_mapper.update(submap)
    global_mapper.update_observer_pos(Point(i, 0))
    visualize_occupancy_grid(global_mapper._occupancy_grid.content)

serialize(global_mapper._occupancy_grid.content)
create_grayscale_bitmap(global_mapper._occupancy_grid.content)

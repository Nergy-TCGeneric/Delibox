from lidar import g2
from mapper import mapper
from typing import List
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
    plt.show()


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
received = g2_lidar.read_data_once(10)
visualize_scan_points(received)

grid_mapper = mapper.Mapper()
grid_mapper.lidar_to_grid(received)
visualize_occupancy_grid(grid_mapper.occupancy_grid)

serialize(grid_mapper.occupancy_grid)
create_grayscale_bitmap(grid_mapper.occupancy_grid)

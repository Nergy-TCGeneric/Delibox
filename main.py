from lidar import g2
from mapper import mapper
from typing import List
import math
import time

import matplotlib.pyplot as plt

def visualize_scan_points(scanned_data: List[g2.LaserScanPoint]):
    fig = plt.figure()
    ax = fig.add_subplot(projection='polar')

    dist = []
    radians = []

    start_time = time.perf_counter_ns()
    for data in scanned_data:
        dist.append(data.distance)
        radians.append(math.radians(data.angle))
    end_time = time.perf_counter_ns()
    print(f"Took {end_time - start_time}ns to finish")

    ax.scatter(radians, dist, s=0.1)
    plt.show()

def visualize_occupancy_grid(grid: List[List[int]]):
    start_time = time.perf_counter_ns() 
    plt.imshow(grid, cmap='binary', vmin=0, vmax=1)
    end_time = time.perf_counter_ns()
    plt.show()
    print(f"Took {end_time - start_time}ns to finish")

port = input("Enter port: ")
g2_lidar = g2.G2(port)
received = g2_lidar.read_data_once(10)
visualize_scan_points(received)

grid_mapper = mapper.Mapper()
grid_mapper.lidar_to_grid(received)
visualize_occupancy_grid(grid_mapper.occupancy_grid)

g2_lidar._stop_scan()
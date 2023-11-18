from lidar import g2
from typing import List
import math

import matplotlib.pyplot as plt

def visualize(scanned_data: List[g2.LaserScanPoint]):
    fig = plt.figure()
    ax = fig.add_subplot(projection='polar')

    dist = []
    radians = []

    for data in scanned_data:
        dist.append(data.distance)
        radians.append(math.radians(data.angle))

    ax.scatter(radians, dist, s=0.1)
    plt.show()

port = input("Enter port: ")
g2_lidar = g2.G2(port)
received = g2_lidar.read_data_once(10)
visualize(received)
g2_lidar._stop_scan()
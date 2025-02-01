#!/usr/bin/env python

import rospy
from nav_msgs.msg import OccupancyGrid
import matplotlib.pyplot as plt
import numpy as np

# Callback function to receive costmap data
def costmap_callback(data):
    # Extract width, height, and map data
    width = data.info.width
    height = data.info.height
    map_data = data.data

    # Convert map data to a 2D numpy array
    costmap = np.array(map_data).reshape((height, width))

    # Plot the costmap with appropriate color mapping
    plt.figure(figsize=(10, 10))
    plt.imshow(costmap, cmap='Blues', origin='lower', vmin=0, vmax=100)
    plt.title('Local Costmap')
    plt.colorbar(label='Occupancy Probability')
    plt.xlabel('Grid cells (x)')
    plt.ylabel('Grid cells (y)')
    plt.grid(True)
    plt.show()

def main():
    rospy.init_node('costmap_reader', anonymous=True)
    rospy.Subscriber('/move_base/local_costmap/costmap', OccupancyGrid, costmap_callback)
    rospy.spin()

if __name__ == '__main__':
    main()

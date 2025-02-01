import matplotlib.patches as patches
import matplotlib.pyplot as plt
import numpy as np
from datetime import datetime

def plot_laser_scan(points, odom_x, odom_y, odom_yaw):
    car_x = odom_x
    car_y = odom_y
    car_direction = odom_yaw

    car_length = 0.508
    car_width = 0.430

    if points:
        x, y = zip(*points)
        plt.scatter(x, y, s=3)

        plt.scatter([car_x], [car_y], color='red')

        rect = patches.Rectangle((car_x - car_length / 2, car_y - car_width / 2), car_length, car_width,
                                 linewidth=1, edgecolor='r', facecolor='none')

        t = patches.transforms.Affine2D().rotate_around(car_x, car_y, car_direction) + plt.gca().transData
        rect.set_transform(t)

        plt.gca().add_patch(rect)

        plt.xlabel('X (map frame)')
        plt.ylabel('Y (map frame)')
        plt.title('Laser Scan Points in Map Frame')
        plt.grid(True)
        plt.gca().set_aspect('equal', adjustable='box')
        plt.gca().set_xticks(np.arange(-5, 5, 1))
        plt.gca().set_yticks(np.arange(-5, 5, 1))

        current_time = datetime.now().strftime('%Y%m%d_%H%M%S')
        filename = f'../figures/laser_scan_{current_time}.png'
        plt.savefig(filename)
        plt.grid(True)
        plt.draw()
        #plt.pause(0.1)
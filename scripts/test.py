import numpy as np
from os.path import join
import rospy
import rospkg
import argparse

def compute_distance(p1, p2):
    return ((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2) ** 0.5


def path_coord_to_gazebo_coord(x, y):
    RADIUS = 0.075
    r_shift = -RADIUS - (30 * RADIUS * 2)
    c_shift = RADIUS + 5

    gazebo_x = x * (RADIUS * 2) + r_shift
    gazebo_y = y * (RADIUS * 2) + c_shift

    return (gazebo_x, gazebo_y)



parser = argparse.ArgumentParser(description='test BARN navigation challenge')
parser.add_argument('--world_idx', type=int, default=2)
parser.add_argument('--gui', type=str, default="false")
parser.add_argument('--out', type=str, default="out.txt")
args = parser.parse_args()
INIT_POSITION = [-2, 3, 1.57]  # in world frame
GOAL_POSITION = [0, 15]

rospack = rospkg.RosPack()
base_path = rospack.get_path('barn_challenge_lu')

path_file_name = join(base_path, "launch/data/path_files", "path_%d.npy" % args.world_idx)
path_array = np.load(path_file_name)
path_array = [path_coord_to_gazebo_coord(*p) for p in path_array]
path_array = np.insert(path_array, 0, (INIT_POSITION[0], INIT_POSITION[1]), axis=0)
path_array = np.insert(path_array, len(path_array),
                       (INIT_POSITION[0] + GOAL_POSITION[0], INIT_POSITION[1] + GOAL_POSITION[1]),
                       axis=0)

print("")

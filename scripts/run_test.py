import time
import argparse
import subprocess
import os
from os.path import join

import numpy as np
import rospy
import rospkg

from gazebo_simulation import GazeboSimulation

INIT_POSITION = [-2, 3, 1.57]  # in world frame
GOAL_POSITION = [0, 15]  # relative to the initial position


def compute_distance(p1, p2):
    return ((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2) ** 0.5


def path_coord_to_gazebo_coord(x, y):
    RADIUS = 0.075
    r_shift = -RADIUS - (30 * RADIUS * 2)
    c_shift = RADIUS + 5

    gazebo_x = x * (RADIUS * 2) + r_shift
    gazebo_y = y * (RADIUS * 2) + c_shift

    return (gazebo_x, gazebo_y)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='test BARN navigation challenge')
    parser.add_argument('--world_idx', type=int, default=0)
    parser.add_argument('--planner', type=str, default="ddp")
    parser.add_argument('--gui', type=str, default="true")
    parser.add_argument('--out', type=str, default="out.txt")
    args = parser.parse_args()

    ##########################################################################################
    ## 0. Launch Gazebo Simulation
    ##########################################################################################

    os.environ["JACKAL_LASER"] = "1"
    os.environ["JACKAL_LASER_MODEL"] = "ust10"
    os.environ["JACKAL_LASER_OFFSET"] = "-0.065 0 0.01"

    if args.world_idx < 300:  # static environment from 0-299
        world_name = "BARN/world_%d.world" % (args.world_idx)
        INIT_POSITION = [-2.25, 3, 1.57]  # in world frame
        GOAL_POSITION = [0, 15]  # relative to the initial position
    elif args.world_idx < 360:  # Dynamic environment from 300-359
        world_name = "DynaBARN/world_%d.world" % (args.world_idx - 300)
        INIT_POSITION = [11, 0, 3.14]  # in world frame
        GOAL_POSITION = [-20, 0]  # relative to the initial position
    else:
        raise ValueError("World index %d does not exist" % args.world_idx)

    print(">>>>>>>>>>>>>>>>>> Loading Gazebo Simulation with %s <<<<<<<<<<<<<<<<<<" % (world_name))
    rospack = rospkg.RosPack()
    base_path = rospack.get_path('barn_challenge_lu')
    os.environ['GAZEBO_PLUGIN_PATH'] = os.path.join(base_path, "plugins")

    launch_file = join(base_path, 'launch', 'gazebo_launch_test.launch')
    world_name = join(base_path, "launch/data/world_files/", world_name)

    gazebo_process = subprocess.Popen([
        'roslaunch',
        launch_file,
        'world_name:=' + world_name,
        'planner:= ' + args.planner,
        'gui:= ' + args.gui
    ])
    
    time.sleep(5)  # sleep to wait until the gazebo being created

    rospy.init_node('gyma', anonymous=True)  # , log_level=rospy.FATAL)
    rospy.set_param('/use_sim_time', True)

    # GazeboSimulation provides useful interface to communicate with gazebo
    gazebo_sim = GazeboSimulation(init_position=INIT_POSITION)

    init_coor = (INIT_POSITION[0], INIT_POSITION[1])
    goal_coor = (INIT_POSITION[0] + GOAL_POSITION[0], INIT_POSITION[1] + GOAL_POSITION[1])

    pos = gazebo_sim.get_model_state().pose.position
    curr_coor = (pos.x, pos.y)
    collided = True

    # check whether the robot is reset, the collision is False
    while compute_distance(init_coor, curr_coor) > 0.1 or collided:
        gazebo_sim.reset()  # Reset to the initial position
        pos = gazebo_sim.get_model_state().pose.position
        curr_coor = (pos.x, pos.y)
        collided = gazebo_sim.get_hard_collision()
        time.sleep(1)

    ##########################################################################################
    ## 1. Launch your navigation stack
    ## (Customize this block to add your own navigation stack)
    ##########################################################################################

    launch_file = join(base_path, "launch/move_base_Lu.launch")
    nav_stack_process = subprocess.Popen([
        'roslaunch',
        launch_file,
    ])

    # Make sure your navigation stack recives the correct goal position defined in GOAL_POSITION
    import actionlib
    from geometry_msgs.msg import Quaternion
    from move_base_msgs.msg import MoveBaseGoal, MoveBaseAction

    nav_as = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
    mb_goal = MoveBaseGoal()
    mb_goal.target_pose.header.frame_id = 'odom'
    mb_goal.target_pose.pose.position.x = GOAL_POSITION[0]
    mb_goal.target_pose.pose.position.y = GOAL_POSITION[1]
    mb_goal.target_pose.pose.position.z = 0
    mb_goal.target_pose.pose.orientation = Quaternion(0, 0, 0, 1)

    nav_as.wait_for_server()
    nav_as.send_goal(mb_goal)


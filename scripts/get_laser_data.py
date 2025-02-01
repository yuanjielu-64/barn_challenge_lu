import rospy
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Path
import matplotlib.pyplot as plt
import numpy as np
from datetime import datetime
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import PoseWithCovarianceStamped
import matplotlib.patches as patches
from ulity import plot_laser_scan
from nav_msgs.msg import Path, Odometry
from tf.transformations import euler_from_quaternion

class LaserScanSubscriber:
    def __init__(self):
        rospy.init_node('laser_scan_subscriber', anonymous=True)

        #self.global_path = rospy.Subscriber("/move_base/TrajectoryPlannerROS/global_plan", Path, self.glpath_callback)
        self.robot_pose_sub = rospy.Subscriber("/odometry/filtered", Odometry, self.robot_pose_callback)
        self.subscriber = rospy.Subscriber('/front/scan', LaserScan, self.scan_callback, queue_size=30)

        self.tfBuffer = tf2_ros.Buffer()
        self.tfListener = tf2_ros.TransformListener(self.tfBuffer)

        self.odom_x = None
        self.odom_y = None
        self.odom_yaw = None
        self.points = []

        rospy.loginfo("Initialization done")


    def robot_pose_callback(self, data):
        try:
            transformStamped = self.tfBuffer.lookup_transform("odom", "base_link", rospy.Time(0))
            quat = transformStamped.transform.rotation
            roll, pitch, yaw = euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])

            self.odom_x = transformStamped.transform.translation.x
            self.odom_y = transformStamped.transform.translation.y
            self.odom_yaw = yaw

            rospy.loginfo("Robot Position in odom (x, y, yaw): (%f, %f, %f)", self.odom_x, self.odom_y, self.odom_yaw)

        except tf2_ros.TransformException as ex:
            rospy.logwarn("Could not lookup transform: %s", ex)

        return data

    def scan_callback(self, msg):
        if self.odom_x is None or self.odom_y is None or self.odom_yaw is None:
            return
        self.points.clear()

        angle = msg.angle_min
        for range in msg.ranges:
            if msg.range_min < range < msg.range_max:
                laser_x = self.odom_x + range * np.cos(angle + self.odom_yaw)
                laser_y = self.odom_y + range * np.sin(angle + self.odom_yaw)
                self.points.append((laser_x, laser_y))
            angle += msg.angle_increment

        plot_laser_scan(self.points, self.odom_x, self.odom_y, self.odom_yaw)


    def glpath_callback(self, data):
        rospy.loginfo("Received the global path of the robot")
        for pose in data.poses:
            x = pose.pose.position.x
            y = pose.pose.position.y
            rospy.loginfo("x: %f, y: %f", x, y)

        return data.poses

    def get_yaw_from_quaternion(self, orientation):
        q = [orientation.x, orientation.y, orientation.z, orientation.w]
        siny_cosp = 2 * (q[3] * q[2] + q[0] * q[1])
        cosy_cosp = 1 - 2 * (q[1] * q[1] + q[2] * q[2])
        return np.arctan2(siny_cosp, cosy_cosp)

    def start(self):
        plt.ion()
        rospy.spin()

if __name__ == '__main__':

    try:
        laser_scan_subscriber = LaserScanSubscriber()
        laser_scan_subscriber.start()
    except rospy.ROSInterruptException:
        pass


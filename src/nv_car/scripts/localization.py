#!/usr/bin/env python3
import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PoseStamped
from tf.transformations import euler_from_quaternion
import sensor_msgs.point_cloud2 as pc2
import math
import numpy as np
import time
import matplotlib.pyplot as plt

class LocalizationController:
    
    def __init__(self):

        self.x0 = None
        self.y0 = None
        self.yaw = None

        # not using map variable
        self.map = None
        self.goal = None

        rospy.init_node('localization_controller', anonymous=True)
        # lidar odom
        rospy.Subscriber('/lio_sam/mapping/odometry', Odometry, self.odometry_callback)
        # map_local / global 
        rospy.Subscriber('/lio_sam/mapping/map_local', PointCloud2, self.pointcloud_callback)
        # get the /gaol topic from rviz 2D nav tool
        rospy.Subscriber('/goal', PoseStamped, self.nav_tool_callback)
        
    
    def odometry_callback(self, msg):
        position = msg.pose.pose.position
        self.x0 = position.x
        self.y0 = position.y

        orientation = msg.pose.pose.orientation
        orientation_list = [orientation.x, orientation.y, orientation.z, orientation.w]
        _, _, yaw_rad = euler_from_quaternion(orientation_list)
        self.yaw = yaw_rad / math.pi * 180.0
    # pointcloud map not used (using rviz)
    def pointcloud_callback(self, msg):
            
        resolution = 0.1
        width = 500
        height = 500
        grid = np.zeros((height, width), dtype=np.int8)
        expansion_radius = 1

        point_cloud = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
        
        # for i, point in enumerate(point_cloud):
        #     x, y = point[0], point[1]
        #     grid_x = int((x + width * resolution / 2) / resolution)
        #     grid_y = int((y + height * resolution / 2) / resolution)

        #     for i in range(-expansion_radius, expansion_radius + 1):
        #         for j in range(-expansion_radius, expansion_radius + 1):
        #             if 0 <= grid_x + i < width and 0 <= grid_y + j < height:
        #                 grid[grid_y + j, grid_x + i] = 100
        #     grid[int(self.x0) + 250, int(self.y0) + 250] = 100

        # plt.matshow(grid)
        # plt.show()

    def nav_tool_callback(self, msg):
        # nav tool fixed frame: map
        position = msg.pose.position
        self.goal = position # position.x position.y position.z
        # send to PID control
            
        print(f"Car pos: ({self.x0:.3f}, {self.y0:.3f}) Goal pos: ({self.goal.x:.3f}, {self.goal.y:.3f})")
        print(f"Distance: {self.calculate_error():.3f}")
        print(f"Angle: {self.calculate_angle():.3f}")

    def calculate_error(self):
        x0, y0 = self.x0, self.y0
        x1, y1 = self.goal.x, self.goal.y

        return ((x0 - x1) ** 2 + (y0 - y1) ** 2) ** 0.5
    
    def calculate_angle(self):
        x0, y0 = self.x0, self.y0
        x1, y1 = self.goal.x, self.goal.y

        v = (x1 - x0, y1 - y0)

        # angle of vector
        angle_v = math.atan2(v[1], v[0]) / math.pi * 180
        # yaw and goal are refered to map fixed frame 
        angle_ya = self.yaw - angle_v
        # angle between yaw and vector -pi ~ pi
        angle_ya = angle_ya if angle_ya > -180.0 else angle_ya + 360.0

        return angle_ya # negative angle --> goal at left / positive --> right
    
    def publish(self):
        pass


if __name__ == '__main__':
    try:
        localization_controller = LocalizationController()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

#!/usr/bin/env python3
import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2,Image
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32, Int8MultiArray, UInt8
from rospy.numpy_msg import numpy_msg
from tf.transformations import euler_from_quaternion
import sensor_msgs.point_cloud2 as pc2
import math
import numpy as np
import time
import cv2
from simple_pid import PID

class LocalizationController:
    
    def __init__(self):

        self.x0 = None
        self.y0 = None
        self.z0 = None
        self.yaw = None

        # not using map variable
        self.map = None
        self.goal = None
        self.controller_goal = None
        self.bridge = CvBridge()
        rospy.init_node('localization_controller', anonymous=True)
        # lidar odom
        rospy.Subscriber('/lio_sam/mapping/odometry', Odometry, self.odometry_callback)
        # map_local / global 
        rospy.Subscriber('/lio_sam/mapping/map_local', PointCloud2, self.pointcloud_callback)
        # get the /goal topic from rviz 2D nav tool
        rospy.Subscriber('/goal', PoseStamped, self.nav_tool_callback)
        # sub controller data for choosing map goal
        rospy.Subscriber('/controller_goal', Int8MultiArray, self.controller_goal_callback)

        self.stall_pid = PID(-9, -0.05, -0.15, setpoint=0)

        self.steer_pid = PID(6, 0.0, 0.0, setpoint=0)

        self.throttle_control = rospy.Publisher('/throttle_pid_control', Float32, queue_size = 10)
        self.steer_control = rospy.Publisher('/steer_pid_control', Float32, queue_size = 10)
        
        self.map_pub = rospy.Publisher('/numpy_map', Image, queue_size=10)

        self.rate = rospy.Rate(10)

    def odometry_callback(self, msg):
        position = msg.pose.pose.position
        self.x0 = position.x
        self.y0 = position.y
        self.z0 = position.z

        orientation = msg.pose.pose.orientation
        orientation_list = [orientation.x, orientation.y, orientation.z, orientation.w]
        _, _, yaw_rad = euler_from_quaternion(orientation_list)
        self.yaw = yaw_rad / math.pi * 180.0
    # pointcloud map not used (using rviz)
    def pointcloud_callback(self, msg):
        z_counter = {}
        # map parameter
        resolution = 0.1
        width = 500
        height = 500
        grid = np.zeros((height, width, 3), dtype=np.uint8) # numpy map
        expansion_radius = 0

        point_cloud = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)

        for i, point in enumerate(point_cloud):
            x, y, z = point[0], point[1], point[2]

            # if z > -4.3:
            #     continue

            grid_x = int((x + width * resolution / 2) / resolution)
            grid_y = int((y + height * resolution / 2) / resolution)

            # if z in z_counter:
            #     z_counter[z] += 1
            # else:
            #     z_counter[z] = 1

            # for z, count in z_counter.items():
            #     print(f"z: {z}, count: {count}")

            for i in range(-expansion_radius, expansion_radius + 1):
                for j in range(-expansion_radius, expansion_radius + 1):
                    if 0 <= grid_x + i < width and 0 <= grid_y + j < height:
                        grid[grid_y + j, grid_x + i, 0] = 255

        # convert controller goal to pointcloud map goal
        if self.controller_goal != None:
            
            c_goal_x, c_goal_y = (-self.controller_goal[0] * 3 + width / 2, -self.controller_goal[1] * 3 + height / 2)
            car_x, car_y = ((self.x0 + width * resolution / 2) / resolution, (self.y0 + height * resolution / 2) / resolution)
            
            converted_x = float(c_goal_x * resolution - (width * resolution / 2))
            converted_y = float(c_goal_y * resolution - (height * resolution / 2))

            # map goal selected by controller
            if self.goal == None:
                self.goal = PoseStamped().pose.position 

            self.goal.x = converted_x
            self.goal.y = converted_y

            car_x = int(car_x)
            car_y = int(car_y)
            c_goal_x = int(c_goal_x)
            c_goal_y = int(c_goal_y)
            # for i in range(-expansion_radius, expansion_radius + 1):
            #     for j in range(-expansion_radius, expansion_radius + 1):
            #         if 0 <= c_goal_x + i < width and 0 <= c_goal_y + j < height:
            #             grid[(c_goal_y + j), (c_goal_x + i), 1] = 255
            #             grid[(car_y + j), (car_x + i), 2] = 255
            # print(f"Car pos: ({self.x0:.3f}, {self.y0:.3f}) Goal pos: ({self.goal.x:.3f}, {self.goal.y:.3f})")
            if 0 <= c_goal_x + i < width and 0 <= c_goal_y + j < height:
                grid[c_goal_y, c_goal_x, 1] = 255
                grid[car_y, car_x, 2] = 255

            grid = np.fliplr(grid)
            rosgrid_image = self.bridge.cv2_to_imgmsg(grid, encoding="rgb8")
            self.map_pub.publish(rosgrid_image)
            # cv2.imshow('map', grid)
            # cv2.waitKey(10)
    def controller_goal_callback(self, msg):
        self.controller_goal = msg.data

    def nav_tool_callback(self, msg):
        # nav tool fixed frame: map
        position = msg.pose.position
        self.goal = position # position.x position.y position.z
        
        print(f"Car pos: ({self.x0:.3f}, {self.y0:.3f}) Goal pos: ({self.goal.x:.3f}, {self.goal.y:.3f})")

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
    
    def publish_control(self):
        
        if self.goal == None:
            self.throttle_control.publish(0)
            self.steer_control.publish(0)

        if self.goal is not None:
            throttle_v = self.stall_pid(self.calculate_error())
            steer_v = self.steer_pid(self.calculate_angle())

            self.throttle_control.publish(throttle_v)
            self.steer_control.publish(steer_v)

            self.rate.sleep()
            # when reaching set a new goal
            if self.calculate_error() <= 0.8:
                self.goal = None
                self.stall_pid.reset()
                self.steer_pid.reset()

if __name__ == '__main__':
    try:
        localization_controller = LocalizationController()
        while not rospy.is_shutdown():
            localization_controller.publish_control()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
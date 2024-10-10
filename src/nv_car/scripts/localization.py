#!/usr/bin/env python3
import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from tf.transformations import euler_from_quaternion
import sensor_msgs.point_cloud2 as pc2

class LocalizationController:
    
    def __init__(self):

        self.x0 = None
        self.y0 = None
        self.yaw = None

        self.map = None
        self.destination = [0, 0]

        rospy.init_node('localization_controller', anonymous=True)
        #lidar odom
        rospy.Subscriber('/lio_sam/mapping/odometry', Odometry, self.odometry_callback)
        #map_local / global 
        rospy.Subscriber('/lio_sam/mapping/map_local', PointCloud2, self.pointcloud_callback)
        
    
    def odometry_callback(self, msg):
        position = msg.pose.pose.position
        self.x0 = position.x
        self.y0 = position.y

        orientation = msg.pose.pose.orientation
        orientation_list = [orientation.x, orientation.y, orientation.z, orientation.w]
        _, _, self.yaw = euler_from_quaternion(orientation_list)

    def pointcloud_callback(self, msg):
        point_cloud = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
        # for i, point in enumerate(point_cloud):
        #     x, y, z = point[:3] 
        #     print(f"Point {i}: x={x}, y={y}, z={z}")
        #     if i > 10:
        #       break
        self.map = point_cloud


    def calculate_error(self):
        x0, y0 = self.x0, self.y0
        x1, y1 = self.destination
        return ((x0 - x1) ** 2 + (y0 - y1) ** 2) ** 0.5
    
    def publish(self):
        pass


if __name__ == '__main__':
    try:
        localization_controller = LocalizationController()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

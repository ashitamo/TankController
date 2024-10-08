#!/usr/bin/env python3
import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2

def odometry_callback(msg):
    position = msg.pose.pose.position
    orientation = msg.pose.pose.orientation
    rospy.loginfo("Odometry - Position: x: %f, y: %f, z: %f", position.x, position.y, position.z)
    rospy.loginfo("Odometry - Orientation: x: %f, y: %f, z: %f, w: %f", orientation.x, orientation.y, orientation.z, orientation.w)

def pointcloud_callback(msg):
    rospy.loginfo("Received PointCloud with width: %d, height: %d", msg.width, msg.height)

def listener():
    rospy.init_node('lio_sam_listener', anonymous=True)

    #lidar odom
    rospy.Subscriber('/lio_sam/mapping/odometry', Odometry, odometry_callback)

    #map_local / global 
    rospy.Subscriber('/lio_sam/mapping/map_local', PointCloud2, pointcloud_callback)

    rospy.spin()

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass

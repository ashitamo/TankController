#!/usr/bin/env python

import rospy
import tf
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from math import sin, cos

class ImuOdom:
    def __init__(self):
        rospy.init_node('imu_odom_node')
        
        self.odom_pub = rospy.Publisher('/odom', Odometry, queue_size=10)
        self.imu_sub = rospy.Subscriber('/imu/data', Imu, self.imu_callback)

        self.br = tf.TransformBroadcaster()
        self.odom_msg = Odometry()
        self.odom_msg.header.frame_id = "odom"
        self.odom_msg.child_frame_id = "base_link"
        
        self.last_time = rospy.Time.now()
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        self.vx = 0.0
        self.vy = 0.0
        self.vth = 0.0

        # For low-pass filter
        self.alpha = 0.1
        self.last_angular_velocity_z = 0.0

    def imu_callback(self, data):
        current_time = rospy.Time.now()
        dt = (current_time - self.last_time).to_sec()

        # Apply low-pass filter to angular velocity
        self.vth = self.alpha * data.angular_velocity.z + (1 - self.alpha) * self.last_angular_velocity_z
        self.last_angular_velocity_z = self.vth

        # Zero-Velocity Update (ZUPT)
        if abs(data.angular_velocity.z) < 0.01 and abs(data.linear_acceleration.x) < 0.01 and abs(data.linear_acceleration.y) < 0.01:
            self.vx = 0.0
            self.vy = 0.0
            self.vth = 0.0

        delta_th = self.vth * dt

        # Integrate acceleration to get velocity
        self.vx += data.linear_acceleration.x * dt
        self.vy += data.linear_acceleration.y * dt

        delta_x = self.vx * cos(self.th) * dt - self.vy * sin(self.th) * dt
        delta_y = self.vx * sin(self.th) * dt + self.vy * cos(self.th) * dt

        self.x += delta_x
        self.y += delta_y
        self.th += delta_th

        # Update orientation
        orientation_quat = tf.transformations.quaternion_from_euler(0, 0, self.th)
        
        self.odom_msg.header.stamp = current_time
        self.odom_msg.pose.pose.position.x = self.x
        self.odom_msg.pose.pose.position.y = self.y
        self.odom_msg.pose.pose.position.z = 0.0
        self.odom_msg.pose.pose.orientation.x = orientation_quat[0]
        self.odom_msg.pose.pose.orientation.y = orientation_quat[1]
        self.odom_msg.pose.pose.orientation.z = orientation_quat[2]
        self.odom_msg.pose.pose.orientation.w = orientation_quat[3]

        self.odom_msg.twist.twist.linear.x = self.vx
        self.odom_msg.twist.twist.linear.y = self.vy
        self.odom_msg.twist.twist.angular.z = self.vth

        self.odom_pub.publish(self.odom_msg)

        self.br.sendTransform((self.x, self.y, 0), 
                              orientation_quat,
                              current_time,
                              "base_link",
                              "odom")

        self.last_time = current_time

if __name__ == '__main__':
    try:
        ImuOdom()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from tf import TransformBroadcaster
from geometry_msgs.msg import Quaternion
import numpy as np

class OdometryPublisher:
    def __init__(self):
        rospy.init_node('pub_odom', anonymous=True)
        
        # Odometry publisher
        self.odom_pub = rospy.Publisher('/odom', Odometry, queue_size=50)
        self.br = TransformBroadcaster()
        # Subscribe to IMU data
        rospy.Subscriber('/imu/data_filtered', Imu, self.imu_callback)
        
        # Initialize variables
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        
        self.vx = 0.0
        self.vy = 0.0
        self.vth = 0.0
        
        self.current_time = rospy.Time.now()
        self.last_time = rospy.Time.now()
        
    def imu_callback(self, data):
        self.current_time = rospy.Time.now()
        
        # Get angular velocity
        self.vth = data.angular_velocity.z if abs(data.angular_velocity.z) > 0.001 else 0.0
        
        # Get linear acceleration and convert to robot's frame of reference

        lax = data.linear_acceleration.x if abs(data.linear_acceleration.x) > 0.001 else 0.0
        lay = data.linear_acceleration.y if abs(data.linear_acceleration.y) > 0.001 else 0.0    

        ax = lax
        ay = lay
        
        # Time difference
        dt = (self.current_time - self.last_time).to_sec()
        
        # Integrate acceleration to get velocity
        self.vx += ax * dt
        self.vy += ay * dt
        
        # Update position
        delta_x = (self.vx * np.cos(self.th) - self.vy * np.sin(self.th)) * dt
        delta_y = (self.vx * np.sin(self.th) + self.vy * np.cos(self.th)) * dt
        delta_th = self.vth * dt
        
        self.x += delta_x
        self.y += delta_y
        self.th += delta_th
        
        # tf
        orientation_quat = quaternion_from_euler(0, 0, self.th)

        # Create odometry message
        odom = Odometry()
        odom.header.stamp = self.current_time
        odom.header.frame_id = "odom"
        
        # Set the position
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = data.orientation
        
        # Set the velocity
        odom.child_frame_id = "base_link"
        odom.twist.twist.linear.x = self.vx
        odom.twist.twist.linear.y = self.vy
        odom.twist.twist.angular.z = self.vth
        
        # Publish the message
        self.odom_pub.publish(odom)
        

        self.br.sendTransform((self.x, self.y, 0), 
                              orientation_quat,
                              self.current_time,
                              "base_link",
                              "odom")

        self.last_time = self.current_time

if __name__ == '__main__':
    try:
        OdometryPublisher()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

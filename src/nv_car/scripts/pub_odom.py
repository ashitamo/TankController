import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import Header
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3, TransformStamped, Transform
from sensor_msgs.msg import Imu
from tf.transformations import euler_from_quaternion, quaternion_from_euler, quaternion_matrix
import tf2_ros


class ImuToOdomConverter:
    def __init__(self):
        rospy.init_node('imu_to_odom', anonymous=True)
        
        self.sub_imu = rospy.Subscriber('/imu/data', Imu, self.imu_callback)
        self.pub_odom = rospy.Publisher('/odom', Odometry, queue_size=10)
        
        self.odom_msg = Odometry()
        self.last_imu_data = None
        self.last_update_time = rospy.Time.now()

        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
    
    def imu_callback(self, imu_msg):
        if self.last_imu_data is None:
            self.last_imu_data = imu_msg
            self.last_update_time = imu_msg.header.stamp
            return

        self.odom_msg.header.frame_id = "map"
        self.odom_msg.child_frame_id = "odom"

        current_time = imu_msg.header.stamp
        dt = (current_time - self.last_update_time).to_sec()

        q = (imu_msg.orientation.x, imu_msg.orientation.y, imu_msg.orientation.z, imu_msg.orientation.w)
        roll, pitch, yaw = euler_from_quaternion(q)

        angular_vel_x = imu_msg.angular_velocity.x
        angular_vel_y = imu_msg.angular_velocity.y
        angular_vel_z = imu_msg.angular_velocity.z

        linear_accel_x = imu_msg.linear_acceleration.x
        linear_accel_y = imu_msg.linear_acceleration.y
        linear_accel_z = imu_msg.linear_acceleration.z

        self.odom_msg.header.stamp = current_time
        self.odom_msg.header.frame_id = "map"
        self.odom_msg.child_frame_id = "odom"

        self.odom_msg.twist.twist.linear.x += dt * linear_accel_x
        self.odom_msg.twist.twist.linear.y += dt * linear_accel_y
        self.odom_msg.twist.twist.linear.z += dt * linear_accel_z

        self.odom_msg.pose.pose.position.x += dt * self.odom_msg.twist.twist.linear.x
        self.odom_msg.pose.pose.position.y += dt * self.odom_msg.twist.twist.linear.y
        self.odom_msg.pose.pose.position.z += dt * self.odom_msg.twist.twist.linear.z

        self.odom_msg.pose.pose.orientation = imu_msg.orientation

        self.odom_msg.twist.twist.angular.x = angular_vel_x
        self.odom_msg.twist.twist.angular.y = angular_vel_y
        self.odom_msg.twist.twist.angular.z = angular_vel_z

        self.pub_odom.publish(self.odom_msg)

        tf = TransformStamped()
        tf.header.stamp = current_time
        tf.header.frame_id = self.odom_msg.header.frame_id
        tf.child_frame_id = self.odom_msg.child_frame_id
        tf.transform.translation.x = self.odom_msg.pose.pose.position.x
        tf.transform.translation.y = self.odom_msg.pose.pose.position.y
        tf.transform.translation.z = self.odom_msg.pose.pose.position.z
        tf.transform.rotation = self.odom_msg.pose.pose.orientation
        self.tf_broadcaster.sendTransform(tf)
    

if __name__ == '__main__':
    try:
        ImuToOdomConverter()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

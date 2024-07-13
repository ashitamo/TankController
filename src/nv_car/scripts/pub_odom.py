import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3

def odom_publisher():
    rospy.init_node('odom_publisher_node', anonymous=True)
    rospy.loginfo("Start odom node")
    pub = rospy.Publisher('odom', Odometry, queue_size=10)
    rate = rospy.Rate(10)  # 10Hz

    odom = Odometry()
    odom.header.frame_id = "odom"
    odom.child_frame_id = "base_link"

    while not rospy.is_shutdown():
        current_time = rospy.Time.now()

        # Update odom header
        odom.header.stamp = current_time

        # Set the position
        odom.pose.pose.position = Point(1.0, 2.0, 0.0)  # Example position (x, y, z)
        
        # Set the orientation
        odom.pose.pose.orientation = Quaternion(0.0, 0.0, 0.0, 1.0)  # Example orientation (quaternion)

        # Set the velocity
        odom.twist.twist = Twist(Vector3(1.0, 0.0, 0.0), Vector3(0.0, 0.0, 0.0))  # Example linear and angular velocity

        # Publish the odom message
        pub.publish(odom)

        rate.sleep()

if __name__ == '__main__':
    try:
        odom_publisher()
    except rospy.ROSInterruptException:
        pass

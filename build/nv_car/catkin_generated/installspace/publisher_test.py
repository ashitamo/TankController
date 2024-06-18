import rospy
from std_msgs.msg import String

def test():
    pub = rospy.Publisher('test_topic', String, queue_size=10)
    rospy.init_node('test_topic', anonymous=True)
    rate = rospy.Rate(1)
    rospy.loginfo("Start publisher node")
    while not rospy.is_shutdown():
        msg = f'Testing - {rospy.get_time()}'
        pub.publish(msg)
        rospy.sleep()

if __name__ == '__main__':
    try: 
        test()
    except rospy.ROSInterruptException:
        pass
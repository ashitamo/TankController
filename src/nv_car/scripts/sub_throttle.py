#! /usr/bin/python3

import rospy
from std_msgs.msg import String,Int8,Int16
import can
import threading


class Throttle:
    bus = None
    def __init__(self):
        super(Throttle, self).__init__()
        self.daemon = True
        bustype = 'socketcan'
        channel = 'can0'
        self._throttle = 0
        self.cid = 0x075
        self.bus = can.Bus(interface=bustype, channel=channel, receive_own_messages=True)
        self.initRos()

    def initRos(self):
        rospy.init_node('throttle_node', anonymous=True)
        rospy.Subscriber("/throttle", Int8, self.callback)

    def callback(self,data):
        
        if data.data>=0 and data.data<50:
            self._throttle = data.data
        elif data.data>50:
            self._throttle = 50
        rospy.loginfo(rospy.get_caller_id() + "throttle %s", self._throttle)

    def run(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            data = [self._throttle, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80]
            msg = can.Message(arbitration_id=self.cid, data=data, is_extended_id=False)
            if self.bus is not None:
                self.bus.send(msg,timeout=0.15)
            rate.sleep()

if __name__ == '__main__':
    try:
        throttle = Throttle()
        throttle.run()
    except rospy.ROSInterruptException:
        pass
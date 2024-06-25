#! /usr/bin/pythpn3

import rospy
from std_msgs.msg import String,Int8,Int16
import can
import threading


class Stall:
    bus = None
    def __init__(self):
        super(Stall, self).__init__()
        self.daemon = True
        bustype = 'socketcan'
        channel = 'can0'
        self._stall = 1
        self.cid = 0x168
        self.bus = can.Bus(interface=bustype, channel=channel, receive_own_messages=True)
        self.initRos()

    def initRos(self):
        rospy.init_node('stall_node', anonymous=True)
        self.rate = rospy.Rate(10)
        rospy.Subscriber("/stall", Int8, self.callback)

    def callback(self,data):
        
        if data.data in [1 ,2, 4, 8]:
            self._stall = data.data
        rospy.loginfo(rospy.get_caller_id() + "stall %s",self._stall)

    def run(self):
        while not rospy.is_shutdown():
            msg = can.Message(arbitration_id=self.cid, data=[self._stall, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80], is_extended_id=False)
            if self.bus is not None:
                self.bus.send(msg,timeout=0.15)
            self.rate.sleep()

if __name__ == '__main__':
    try:
        stall = Stall()
        stall.run()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
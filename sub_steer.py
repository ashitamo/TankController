#! /usr/bin/python3
import rospy
from std_msgs.msg import String,Int8,Int16
import can
import threading


class Steer():
    bus = None
    def __init__(self):
        super(Steer, self).__init__()
        self.daemon = True
        bustype = 'socketcan'
        channel = 'can0'
        self._steer = 9000
        self.cid = 0x065
        self.bus = can.Bus(interface=bustype, channel=channel, receive_own_messages=True)
        self.initRos()

    def initRos(self):
        rospy.init_node('steer_node', anonymous=True)
        self.rate = rospy.Rate(10)
        rospy.Subscriber("/steer", Int16, self.callback)

    def callback(self,data):
        rospy.loginfo(rospy.get_caller_id() + "steer %s", data.data)
        if data.data>=0 and data.data<=18000:
            self._steer = data.data

    def run(self):
        while not rospy.is_shutdown():
            valLowByte = self._steer & 0xFF
            valHighByte = (self._steer >> 8) & 0xFF
            data = [valLowByte, valHighByte, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80]
            msg = can.Message(arbitration_id=self.cid, data=data, is_extended_id=False)
            if self.bus is not None:
                self.bus.send(msg,timeout=0.15)
            self.rate.sleep()

if __name__ == '__main__':
    try:
        steer = Steer()
        steer.run()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
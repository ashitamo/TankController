#! /usr/bin/pythpn3
import rospy
from std_msgs.msg import String,Int8,Int16
import can
import threading


class Heartbeam():
    bus = None
    def __init__(self):
        super(Heartbeam, self).__init__()
        self.daemon = True
        bustype = 'socketcan'
        channel = 'can0'
        self._heartbeam = 0
        self.cid = 0x43f
        self.bus = can.Bus(interface=bustype, channel=channel, receive_own_messages=True)
        self.initRos()

    def initRos(self):
        rospy.init_node('heartbeam_node', anonymous=True)
        self.rate = rospy.Rate(10)

    def run(self):
        while not rospy.is_shutdown():
            if self._heartbeam > 255:
                self._heartbeam = 0
            data = [0x00, 0x00, 0x00, 0x00, 0x0F, 0x00, 0x00, 0x80] 
            msg = can.Message(arbitration_id=self.cid, data=data,is_extended_id=False)
            if self.bus is not None:
                self.bus.send(msg,timeout=0.15)
            self._heartbeam += 1
            #rospy.loginfo(rospy.get_caller_id() + "heartbeam %d", self._heartbeam)
            self.rate.sleep()

if __name__ == '__main__':
    try:
        heartbeam = Heartbeam()
        heartbeam.run()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
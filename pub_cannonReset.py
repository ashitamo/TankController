#! /usr/bin/python3
import socket
import threading
import time
import json
import queue
import rospy
import math
from std_msgs.msg import String,Int8,Int16


if __name__ == '__main__':
    rospy.init_node('cannon_reset_node', anonymous=True)
    commandPublisher = rospy.Publisher("/cannon_reset",String,queue_size=10)
    while True:
        print("type 'b' to reset base, 'f' to reset fort")
        command = input()
        if command == 'b':
            command = "reset_base"
        elif command == 'f':
            command = "reset_fort"
        else:
            continue
        commandPublisher.publish(command)
       
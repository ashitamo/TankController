#! /usr/bin/python3
import socket
import threading
import time
import json
import queue
import random
import can
import rospy
from std_msgs.msg import String,Int8,Int16,UInt8
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from rospy.numpy_msg import numpy_msg
import numpy as np

import simplejpeg

HOST = "127.0.0.1"
HOST = "10.147.18.60"
#HOST = "192.168.0.157"
#HOST = "10.22.233.150"
PORT = 65321


class CarMapEmiter(threading.Thread):
    socket = None
    def __init__(self):
        super().__init__()
        self.daemon = True

        rospy.init_node('carMapEmiter_node', anonymous=True)
        self.bridge = CvBridge()
        self.initSocket() 
        rospy.Subscriber("/numpy_map", Image, self.callback_map)
        self.map = simplejpeg.encode_jpeg(np.ones((500,500,3), dtype=np.uint8)*255, colorspace='RGB')
    def callback_map(self,data):
        grid = self.bridge.imgmsg_to_cv2(data, desired_encoding="rgb8")
        self.map = simplejpeg.encode_jpeg(grid, colorspace='RGB',colorsubsampling='444',quality=85)

    def initSocket(self):
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        
    def connecting(self):
        while True:
            if self.socket is None:
                self.initSocket()
            self.socket.settimeout(1)
            try:
                self.socket.connect((HOST, PORT))
                print("connected")
                break
            except ConnectionRefusedError:
                print("Connection refused")
            except TimeoutError:
                print("waiting connection time out")
            except BaseException as e:
                print(e)
                self.socket = None
                continue
        self.socket.settimeout(None)  

    def emit(self):
        self.socket.settimeout(0.5)
        data = self.map
        try:
            print(len(data))
            self.socket.sendall(data)
        except TimeoutError:
            return None
        except BaseException as e:
            if "Errno 32" in str(e):
                self.socket.close()
                self.socket = None
            return None
        return True
    
    def run(self):
        self.connecting()   
        while True:
            sucess = self.emit()
            if self.socket is None:
                self.connecting()
                continue
            time.sleep(1)


if __name__ == "__main__":
    try:
        carMapEmiter = CarMapEmiter()
        carMapEmiter.start()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    
        

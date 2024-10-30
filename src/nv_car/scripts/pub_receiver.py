#! /usr/bin/python3

import socket
import threading
import time
import json
import queue
import rospy
import math
import random
from std_msgs.msg import String, Int8, Int16, Float32

HOST = "10.147.18.60"
#HOST = "192.168.0.157"
PORT = 65432

class Receiver(threading.Thread):
    socket = None
    def __init__(self):
        self.initSocket() 
        super().__init__()
        self.daemon = True
        self.rosQueue = queue.Queue(1)
        self.totalCount = 1
        self.failCount = 0
        self.recvTimeoutDuration = 0.15
    
    def initSocket(self):
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

    def connecting(self):
        
        while True:
            if self.socket is None:
                self.initSocket()
            self.socket.settimeout(5)
            try:
                self.socket.connect((HOST, PORT))
                print("connected")
                break
            except ConnectionRefusedError:
                print("Connection refused")
            except TimeoutError:
                print("time out")
            except BaseException as e:
                print(e)
                self.socket = None
                continue
        self.recvTimeoutDuration = 0.15
        self.socket.settimeout(None)  

    def recive(self):
        if self.totalCount> 100:
            self.totalCount = 50
            self.failCount = self.failCount % 25
        self.recvTimeoutDuration = 0.15 + (self.failCount/self.totalCount)*100* 0.01
        #print(self.recvTimeoutDuration)
        if self.recvTimeoutDuration > 0.75:
            self.recvTimeoutDuration = 0.75
        elif self.recvTimeoutDuration < 0.15:
            self.recvTimeoutDuration = 0.15
        self.socket.settimeout(self.recvTimeoutDuration)  
        #print(self.recvTimeoutDuration)

        try:
            self.totalCount+=1
            data = self.socket.recv(90)
        except TimeoutError:
            self.failCount+=1
            return None
        except BaseException as e:
            #print(e)
            if "WinError 10054" in str(e):
                self.socket = None
            return None
        if len(data) == 0:
            self.socket.close()
            self.socket = None
            return None
        data = data.decode("utf-8")
        try:
            data = json.loads(data)
        except:
            self.failCount+=1
            return None
        return data
    
    def run(self):
        self.connecting()   
        while True:
            data = self.recive()
            if self.socket is None:
                self.connecting()
                continue
            try:
                self.rosQueue.put(data,block=False,timeout=0.1)
            except queue.Full:
                pass
            

class rosPublisher:
    def __init__(self):
        self.stallPublisher = rospy.Publisher("/stall",Int8,queue_size=10)
        self.throttlePublisher = rospy.Publisher("/throttle",Int8,queue_size=10)
        self.steerPublisher = rospy.Publisher("/steer",Int16,queue_size=10)
        self.cannonPublisher = rospy.Publisher("/cannon",String,queue_size=10)
        self.lastData = None
        self.count = 0

        self.pid_throttle = None
        self.pid_steer = None
        rospy.Subscriber("/throttle_pid_control", Float32, self.throttle_pid_callback)
        rospy.Subscriber("/steer_pid_control", Float32, self.steer_pid_callback)

    def throttle_pid_callback(self, data):
        value = data.data
        if value > 30:
            value = 30
        self.pid_throttle = int(value)

    def steer_pid_callback(self, data):
        value = data.data
        self.pid_steer = int(value) + 9000

    def attenuate(self,count):
        '''
            輸出資料格式
            throttle: -1000~1000
            steer: -1000~1000

            斷訊時方向盤自動回正
            檔位保持上一個檔位
            油門歸零
        '''
        attData = {"throttle":0,"steer":0,}
        if self.lastData is None:
            return attData
        attData = self.lastData
        if count>=3:
            attData['m'] = 1
            attData['g'] = [None,None]
        steer = self.lastData["steer"]
        steer = int((steer)*math.exp(-count/15))
        throttle = self.lastData["throttle"]
        throttle = int((throttle)*math.exp(-count/8))
        # if self.lastData["throttle"] < 0:
        #     attData["throttle"] = -1
        attData["throttle"] = throttle
        attData["steer"] = steer
        return attData
    def convert(self,data):
        '''
            轉換訊號用

            輸入資料格式
            throttle: -1000~1000
            steer: -1000~1000
            stall: 1 2 4 8
            base: -60~60
            fort : -40~10
            manual: 1 || 0
            goal: [-100~100,-100~100] || [None,None](when not recv, the unit is %

            輸出資料格式
            throttle: 0~40(stall = 1) 0~60(stall = 2)
            steer: 1500~14500
            stall: 1 2 4 8
            base: -60~60
            fort : -40~10
            manual: True || False
            goal: [-100~100,-100~100] || [None,None](when not recv, the unit is %
        '''
        if data is None:
            data = self.attenuate(self.count)
            #data = {"throttle":0,"steer":0,"stall":1}
            self.count+=1
        else:
            self.count = 0
        self.lastData = data.copy()

        if data["throttle"] >= 0:
            stall = 1
            data["throttle"] = abs(data["throttle"]*40/1000)
        elif data["throttle"] < 0:
            stall= 2
            data["throttle"] = abs(data["throttle"]*60/1000)
        
        data["steer"] = ((data["steer"]+1000)/2000)*1000+8500
        throttle = int(data["throttle"])
        steer = int(data["steer"])
        cannon_cmd = ''
        if 'base' in data.keys() and 'fort' in data.keys():
            cannon_cmd = '{},{}'.format(data['base'],data['fort'])
        if 'launch' in data.keys():
            cannon_cmd = 'launch'
        manual = True
        if 'm' not in data.keys():#if controller is disconnect, disable autocontrol
            manual= True
            goal = [None,None]
        else:
            manual = True if data['m']== 1 else False
            goal = data['g'] 
        data = {
            "throttle":throttle,
            "steer":steer,
            "stall":stall,
            'cannon': cannon_cmd,
            'manual':manual,
            'goal':goal
        }
        if manual == False:
            data["throttle"] = self.pid_throttle
            data["steer"] = self.pid_steer

        return data # adding control mode status
    
    def publish(self,data):
        self.stallPublisher.publish(data["stall"])
        self.throttlePublisher.publish(data["throttle"])
        self.steerPublisher.publish(data["steer"])
        self.cannonPublisher.publish(data['cannon'])
    
if __name__ == "__main__":
    receiver = Receiver()
    receiver.start()
    rospy.init_node('receiver_node', anonymous=True)
    rate = rospy.Rate(10)
    publisher = rosPublisher()
    while not rospy.is_shutdown():
        if not receiver.rosQueue.empty():
            data = receiver.rosQueue.get() # controller data
        else:
            data = None
        data = publisher.convert(data) # data , status
        print(data)
        publisher.publish(data)
        rate.sleep()
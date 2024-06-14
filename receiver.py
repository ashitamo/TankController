import socket
import threading
import time
import json
import queue
import rospy
import math
from std_msgs.msg import String,Int8,Int16


HOST = "10.147.18.60"
PORT = 65432

class Receiver(threading.Thread):
    socket = None
    def __init__(self):
        self.initSocket() 
        super().__init__()
        self.daemon = True
        self.rosQueue = queue.Queue(1)
    
    def initSocket(self):
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

    def connecting(self):
        
        while True:
            if self.socket is None:
                self.initSocket()
            self.socket.settimeout(10)
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
        self.socket.settimeout(None)  

    def recive(self):
        self.socket.settimeout(0.15)
        try:
            data = self.socket.recv(40)
        except TimeoutError:
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
        except json.decoder.JSONDecodeError:
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
        self.lastData = None
        self.count = 0

    def attenuate(self,count):
        '''
            斷訊時方向盤自動回正
            檔位保持上一個檔位
            油門歸零
        '''
        attData = {"throttle":0,"steer":0,"stall":1}
        if self.lastData is None:
            return attData
        
        attData["stall"] = self.lastData["stall"]
        steer = self.lastData["steer"]
        steer = int((steer)*math.exp(-count/2))
        attData["steer"] = steer
        attData["stall"] = self.lastData["stall"]
        return attData
    def convert(self,data):
        '''
            轉換訊號用

            輸入資料格式
            throttle: -1000~1000
            steer: -1000~1000
            stall: 1 2 4 8

            輸出資料格式
            throttle: 0~30(stall = 1) 0~60(stall = 2)
            steer: 0 ~ 18000
            stall: 1 2 4 8
        '''
        if data is None:
            #data = self.attenuate(self.count)
            data = {"throttle":0,"steer":0,"stall":1}
            self.count+=1
        else:
            self.count = 0

        if data["throttle"] > 0:
            stall = 1
            data["throttle"] = abs(data["throttle"]*30/1000)
        elif data["throttle"] < 0:
            stall= 2
            data["throttle"] = abs(data["throttle"]*60/1000)

        data["steer"] = ((data["steer"]+1000)/2000)*18000
        throttle = int(data["throttle"])
        steer = int(data["steer"])
        data = {"throttle":throttle,"steer":steer,"stall":stall}
        self.lastData = data
        return data
    
    def publish(self,data):
        self.stallPublisher.publish(data["stall"])
        self.throttlePublisher.publish(data["throttle"])
        self.steerPublisher.publish(data["steer"])
    
if __name__ == "__main__":
    receiver = Receiver()
    receiver.start()
    rospy.init_node('receiver_node', anonymous=True)
    rate = rospy.Rate(10)
    publisher = rosPublisher()
    while not rospy.is_shutdown():
        try:
            data = receiver.rosQueue.get(block=False,timeout=0.1)
        except queue.Empty:
            data = None
        data = publisher.convert(data)
        print(data)
        publisher.publish(data)
        rate.sleep()
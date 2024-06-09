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

    def attenuate(self,data,count):
        '''
            斷訊時方向盤自動回正
        '''
        attData = {"throttle":0,"steer":9000,"stall":1}
        if data is None:
            return attData
        throttle = data["throttle"]
        steer = data["steer"]
        steer = int((steer-9000)*math.exp(-count/2))+9000
        attData["steer"] = steer
        attData["stall"] = data["stall"]
        return attData
    def convert(self,data):
        '''
            轉換訊號用

            輸入資料格式
            throttle: -1000~1000
            steer: -1000~1000
            stall: 1 2 4 8

            輸出資料格式
            throttle: 0~30(stall = 1) 0~50(stall = 2)
            steer: 0 ~ 18000
            stall: 1 2 4 8
        '''
        if data is None:
            data = self.attenuate(self.lastData,self.count)
            self.count+=1
        elif data["throttle"] < -1000 or data["throttle"] > 1000:
            data = self.attenuate(self.lastData,self.count)
            self.count+=1
        elif data["steer"] < -1000 or data["steer"] > 1000:
            data = self.attenuate(self.lastData,self.count)
            self.count+=1
        else:
            self.count = 0

        if data["throttle"] >= 0:
            stall = 1
            data["throttle"] = abs(data["throttle"]*30/1000)
        else:
            stall = 2
            data["throttle"] = abs(data["throttle"]*50/1000)
        data["steer"] = (data["steer"]+1000)/2000*18000
        throttle = int(data["throttle"])
        steer = int(data["steer"])
        data = {"throttle":throttle,"steer":steer,"stall":stall}
        self.lastData = data
        return data
    
    def publish(self,data):
        data = self.convert(data)
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
            data = publisher.convert(data)
            publisher.publish(data)
            print(data)
        except queue.Empty:
            pass
        rate.sleep()
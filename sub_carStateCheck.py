#! /usr/bin/python3
import socket
import threading
import time
import json
import queue
import random
import can
#import rospy
#from std_msgs.msg import String,Int8,Int16

HOST = "127.0.0.1"
HOST = "10.147.18.60"
#HOST = "10.22.233.150"
PORT = 65434

class CarStateReader(threading.Thread):
    ESP_VOLT = -1
    THROTTLE = -1
    STEER = -1
    STALL = -1
    SPEED = -1

    def __init__(self):
        super().__init__()
        self.daemon = True
        self.bus = can.Bus(interface='socketcan', channel='can0', receive_own_messages=True)

    def readBus(self):
        msg = self.bus.recv()
        if msg.arbitration_id == 0x363:
            self.SPEED = msg.data[0]+msg.data[1]*256
        elif msg.arbitration_id == 0x0A3:
            self.STALL = msg.data[0]
        elif msg.arbitration_id == 0x067:
            self.ESP_VOLT = msg.data[1]/100*12
            self.STEER = msg.data[2]+msg.data[3]*256
        elif msg.arbitration_id == 0x50:
            self.THROTTLE = msg.data[0]

    def run(self):
        while True: 
            self.readBus()
            time.sleep(0.01)


class CarStateChecker_Recv(threading.Thread):
    socket = None
    def __init__(self):
        super().__init__()
        self.initSocket() 
        self.daemon = True
        self.StateReader = CarStateReader()
        self.StateReader.start()
    
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

    def respond(self):
        self.socket.settimeout(0.15)
        try:
            data = self.socket.recv(29)
        except TimeoutError:
            return None
        except BaseException as e:
            print(e)
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
            data["ESP_VOLT"] = self.StateReader.ESP_VOLT
            data["THROTTLE"] = self.StateReader.THROTTLE
            data["STEER"] = self.StateReader.STEER
            data["STALL"] = self.StateReader.STALL
            data["SPEED"] = self.StateReader.SPEED
        except json.decoder.JSONDecodeError:
            return None
        try:
            data = json.dumps(data).encode("utf-8")
            print(len(data))
            self.socket.sendall(data)
        except TimeoutError:
            return None
        except BaseException as e:
            print(e)
            return None
        
        return data
    
    def run(self):
        self.connecting()   
        while True:
            data = self.respond()
            if self.socket is None:
                self.connecting()
                continue


if __name__ == "__main__":
    checker = CarStateChecker_Recv()
    checker.start()
    while True:
        time.sleep(0.1)
    
        
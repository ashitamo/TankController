#! /usr/bin/python3
import socket
import threading
import time
import json
import queue
import random
#import rospy
#from std_msgs.msg import String,Int8,Int16

HOST = "127.0.0.1"
HOST = "10.147.18.60"
#HOST = "10.22.233.150"
PORT = 65434

class CarStateChecker_Recv:
    socket = None
    def __init__(self):
        self.initSocket() 
        self.daemon = True
    
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
                print("time out")
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
        except json.decoder.JSONDecodeError:
            return None
        try:
            self.socket.sendall(json.dumps(data).encode("utf-8"))
        except TimeoutError:
            return None
        except BaseException as e:
            print(e)
            return None
        
        return data
    
    def run(self):
        self.connecting()   
        while True:
            try:
                data = self.respond()
                if self.socket is None:
                    self.connecting()
                    continue
            except KeyboardInterrupt:
                self.socket.close()
                break
            


if __name__ == "__main__":
    checker = CarStateChecker_Recv()
    checker.run()
    
        
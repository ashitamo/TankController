import socket
import threading
import time
import json
import queue
import serial
from pynput import keyboard
import random

HOST = "10.147.18.60"
HOST = "10.22.233.150"
#HOST = "127.0.0.1"
PORT = 65432

"-vcodec libx265 -crf 18"

class Emiter(threading.Thread):
    socket = None
    client = None
    def __init__(self):
        self.initSocket() 
        super().__init__()
        self.daemon = True
        self.timeoutCount = 0
        self.emitQueue = queue.Queue(1)
    def initSocket(self):
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.socket.bind((HOST, PORT))
        self.socket.listen(1)

    def waitClient(self):
        self.socket.settimeout(0.5)
        while True:
            try:
                self.client, addr = self.socket.accept()
                print("Connected by", addr)
                break
            except TimeoutError:
                print("time out")
            except BaseException as e:
                print(e)

    def emit(self):
        try:
            data  = self.emitQueue.get(False,1)
        except queue.Empty:
            return None
        try:
            data = self.client.sendall(data)
            self.timeoutCount = 0
            return True
        except TimeoutError:
            print("send time out")
            self.timeoutCount += 1
            if self.timeoutCount > 10:
                self.client.close()
                self.client = None
            return None
        except BaseException as e:
            if "Errno 32" in str(e):
                self.client.close()
                self.client = None
            return None
        
    
    def run(self):
        self.waitClient()
        while True:
            sucess = self.emit()
            if self.client is None:
                self.waitClient()
            time.sleep(0.05)





if __name__ == '__main__':
    currently_pressed = set()
    W = set(['w'])
    S = set(['s'])
    A = set(['a'])
    D = set(['d'])
    WD = set(['w','d'])
    WA = set(['w','a'])
    SD = set(['s','d'])
    SA = set(['s','a'])
    def on_release(key):
        try:
            currently_pressed.remove(key.char)
        except AttributeError:
            if key == keyboard.Key.space:
                currently_pressed.remove(key)
            pass

    def on_press(key):
        try:
            currently_pressed.add(key.char)
        except AttributeError:
            if key == keyboard.Key.space:
                currently_pressed.add(key)
            pass
 
    listener = keyboard.Listener(on_press=on_press, on_release=on_release)
    listener.start()
    emiter = Emiter()
    emiter.start()  
    while True:
        if currently_pressed == W:
            data = json.dumps({"throttle":1000,"steer":0}).encode("utf-8")

            print('w')
        elif currently_pressed == S:
            data = json.dumps({"throttle":-1000,"steer":0}).encode("utf-8")

            print('s')
        elif currently_pressed == A:
            data = json.dumps({"throttle":0,"steer":1000}).encode("utf-8")

            print('a')
        elif currently_pressed == D:
            data = json.dumps({"throttle":0,"steer":-1000}).encode("utf-8")

            print('d')
        elif currently_pressed == WD:
            data = json.dumps({"throttle":1000,"steer":-1000}).encode("utf-8")

            print('wd')
        elif currently_pressed == WA:
            data = json.dumps({"throttle":1000,"steer":1000}).encode("utf-8")

            print('wa')
        elif currently_pressed == SD:
            data = json.dumps({"throttle":-1000,"steer":-1000}).encode("utf-8")

            print('sd')
        elif currently_pressed == SA:
            data = json.dumps({"throttle":-1000,"steer":1000}).encode("utf-8")
            print('sa')
        else:
            data = json.dumps({"throttle":0,"steer":0}).encode("utf-8")
        if keyboard.Key.space in currently_pressed:
            data = json.dumps({"throttle":0,"steer":0}).encode("utf-8")
            print('space')
        print(data)
        if data is not None:
            try:
                # if random.random() > 0.90:
                #     data = ''
                emiter.emitQueue.put(data, False,0.01)
            except queue.Full:
                pass
        time.sleep(0.1)
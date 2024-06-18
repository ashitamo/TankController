import pygame
from emiter_controller import Emiter
import socket
import threading
import time
import json
import queue
import serial
from pynput import keyboard
pygame.init()
pygame.joystick.init()

joystick = pygame.joystick.Joystick(0)
joystick.init()

print(joystick.get_name())

# axes
# 0 steering wheel | 1 2 3 right/mid/left pedal

# buttons
# 0 1 2 3 cross square circle triangle
# 4 5 steering wheel button right/left

# dpad
# (-1, 1) (0, 1) (1, 1)
# (-1, 0) (0, 0) (1, 0)
# (-1,-1) (0,-1) (1,-1)

keyDict = {'Steering Wheel':0,
            'Right Pedal':1,
            'Middle Pedal':1,
            'Left Pedal':1,
            'Cross':0,
            'Square':0,
            'Circle':0,
            'Triangle':0,
            'Steering Wheel Button Right':0,
            'Steering Wheel Button Left':0,
            'Dpad':(0, 0)}

def controller_key():

    for event in pygame.event.get():
        if event.type == pygame.JOYAXISMOTION:

            keyValue = round(event.value, 3)

            if event.axis == 0:
                keyDict['Steering Wheel'] = keyValue
            elif event.axis == 5:
                keyDict['Right Pedal'] = keyValue
            elif event.axis == 2:
                keyDict['Middle Pedal'] = keyValue
            elif event.axis == 4:
                keyDict['Left Pedal'] = keyValue

        if event.type == pygame.JOYBUTTONDOWN:
            
            keyValue = 1

            if event.button == 0:
                keyDict['Cross'] = keyValue
            elif event.button == 4:
                keyDict['Square'] = keyValue
            elif event.button == 5:
                keyDict['Circle'] = keyValue
            elif event.button == 3:
                keyDict['Triangle'] = keyValue
            elif event.button == 1:
                keyDict['Steering Wheel Button Right'] = keyValue
            elif event.button == 2:
                keyDict['Steering Wheel Button Left'] = keyValue

        if event.type == pygame.JOYBUTTONUP:
            
            keyValue = 0

            if event.button == 0:
                keyDict['Cross'] = keyValue
            elif event.button == 4:
                keyDict['Square'] = keyValue
            elif event.button == 5:
                keyDict['Circle'] = keyValue
            elif event.button == 3:
                keyDict['Triangle'] = keyValue
            elif event.button == 1:
                keyDict['Steering Wheel Button Right'] = keyValue
            elif event.button == 2:
                keyDict['Steering Wheel Button Left'] = keyValue

        if event.type == pygame.JOYHATMOTION:

            keyDict['Dpad'] = event.value

    return keyDict

emiter = Emiter()
emiter.start()  
stall = 1
if __name__ == '__main__':
    while True:
        temp = controller_key()
        if int(temp["Steering Wheel Button Left"]) == 1:
            stall = 2
        if int(temp["Steering Wheel Button Right"]) == 1:
            stall = 1
        throttle = float(temp["Right Pedal"]+1)/2*1000
        if stall == 2:
            throttle = -throttle
        steer = float(temp["Steering Wheel"])*1000
        if abs(steer) < 11.2: #死區 小於兩度為零
            steer = 0
        data = {"throttle":int(throttle),"steer":int(steer)}
        print(data)
        try:
            emiter.emitQueue.put(json.dumps(data).encode("utf-8"), False)
        except queue.Full:
                pass
        time.sleep(0.1)
        
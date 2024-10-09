#! /usr/bin/python3

import rospy
from std_msgs.msg import String,Int8,Int16
import can
import threading
import serial
import time
import queue

# Serial Port Information
SERIAL_PORT = '/dev/ttyACM0'  # Update this to your port
BAUD_RATE = 57600
# Global Serial Object

class Arduino:
    def __init__(self):
        self.serial = None
        self.open_serial_connection()
        self.recvThread = threading.Thread(target=self.recv)
        self.recvThread.daemon = True
        self.recvFlag = True
        self.recvThread.start()
        
        self.base_target_angle = 0
        self.base_angle_input = 0

        self.fort_target_angle = 0
        self.fort_angle_input = 0
        self.status = 0
    def close_serial_connection(self):
        self.recvFlag = False
        self.serial.close()
        print("Shutting down")
    # Function to open/reopen the serial connection
    def open_serial_connection(self):
        while True:
            try:
                self.serial = serial.Serial(port=SERIAL_PORT, baudrate=BAUD_RATE, timeout=None)
                time.sleep(2)  # Allow time for the connection to stabilize
                print("Serial connection established")
                return
            except serial.SerialException:
                print("Failed to connect. Retrying in 2 seconds...")
                time.sleep(2)

    # ===============================
    # Serial Communication Functions with Reconnect Logic
    # ===============================
    def send_serial_command(self,command):
        """Send command to Arduino via serial and handle disconnection."""
        try:
            if self.serial is None or not self.serial.is_open:
                self.open_serial_connection()
            # self.serial.flush()
            self.serial.write(command.encode())
            time.sleep(0.01)
        except serial.SerialException:
            print("Serial connection lost. Reconnecting...")
            self.open_serial_connection()
            self.serial.write(command.encode())  # Resend the command after reconnect
            
    def launch(self):
        command = "LAUNCH\n"
        self.send_serial_command(command)

    def set_target_angle(self,motor, angle):
        command = f"SET_{motor}:{angle}\n"
        self.send_serial_command(command)

    def reset_motor(self,motor):
        command = f"RESET_{motor}\n"
        self.send_serial_command(command)

    def reset_motor_switch(self,motor):
        command = f"RESET_MS_{motor}\n"
        self.send_serial_command(command)
        time.sleep(3)
        while self.status != 0:
            time.sleep(0.1)
        # Wait for the Arduino to reset or stabilize
          # Adjust this delay based on how long the reset takes
        if self.serial.is_open:
            return
        # Try reconnecting the serial connection
        for _ in range(5):  # Retry 5 times
            try:
                self.open_serial_connection()
                print(f"Reconnected to {motor} after reset.")
                break
            except serial.SerialException:
                print(f"Failed to reconnect to {motor}. Retrying...")
                time.sleep(2)

    def recv(self):
        while self.recvFlag:
            if self.serial.in_waiting:
                response = self.serial.readline().decode('utf-8', errors='ignore').strip()
                response = response.split(",")
                if len(response) != 6:
                    continue
                try:
                    self.status = int(response[0])
                    self.base_target_angle = float(response[1])
                    self.fort_target_angle= float(response[3])
                    print(response)
                except:
                    pass

class Cannon(threading.Thread):
    bus = None
    def __init__(self):
        super(Cannon, self).__init__()
        self.daemon = True
        self.arduino = Arduino()
        self.base = 0
        self.fort = 0
        self.reset_ms_base = False
        self.reset_ms_fort = False
        self.launchQueue = queue.Queue(1)
        self.initRos()

    def initRos(self):
        rospy.init_node('cannon_node', anonymous=True)
        self.rate = rospy.Rate(10)
        rospy.Subscriber("/cannon", String, self.callback)
        rospy.Subscriber("/cannon_reset", String, self.callback_reset)
    def callback_reset(self,data):
        msg = data.data
        if msg == "reset_fort":
            self.reset_ms_base = True
        elif msg == "reset_base":
            self.reset_ms_fort = True
    def callback(self,data):
        msg = data.data.split(",")
        if len(msg) == 2:
            self.base = int(msg[0])
            self.fort = int(msg[1])
        elif data.data == "launch":
            if not self.launchQueue.full():
                self.launchQueue.put(True)
        #rospy.loginfo(rospy.get_caller_id() + "stall %s",self._stall)

    def run(self):
        b = True
        while not rospy.is_shutdown():
            if self.reset_ms_base:
                self.arduino.reset_motor_switch("BASE")
                self.reset_ms_base = False
            elif self.reset_ms_fort:
                self.arduino.reset_motor_switch("FORT")
                self.reset_ms_fort = False
            print(self.fort,self.base,self.arduino.fort_angle_input,self.arduino.base_angle_input)
            if not self.launchQueue.empty():
                self.launchQueue.get()
                self.arduino.launch()
            if self.fort != self.arduino.fort_target_angle:
                self.arduino.set_target_angle("FORT", self.fort)
            if self.base != self.arduino.base_target_angle:
                self.arduino.set_target_angle("BASE", self.base)

            self.rate.sleep()
        self.arduino.close_serial_connection()
        

if __name__ == '__main__':
    try:
        Cannon = Cannon()
        Cannon.run()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
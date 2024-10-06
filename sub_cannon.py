#! /usr/bin/python3

import rospy
from std_msgs.msg import String,Int8,Int16
import can
import threading
import serial
import time

# Serial Port Information
SERIAL_PORT = '/dev/ttyACM0'  # Update this to your port
BAUD_RATE = 5760
# Global Serial Object

class Arduino:
    def __init__(self):
        self.serial = None
        self.open_serial_connection()
        self.base_angle_input = 0
        self.fort_angle_input = 0


    # Function to open/reopen the serial connection
    def open_serial_connection(self):
        while True:
            try:
                self.serial = serial.Serial(port=SERIAL_PORT, baudrate=BAUD_RATE, timeout=1)
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
            self.serial.write(command.encode())
            time.sleep(0.01)
        except serial.SerialException:
            print("Serial connection lost. Reconnecting...")
            self.open_serial_connection()
            self.serial.write(command.encode())  # Resend the command after reconnect
            
    def get_current_angle(self,motor):
        """Request and retrieve the current angle for the specified motor."""
        command = f"GET_{motor}\n"
        self.send_serial_command(command)
        #time.sleep(0.01)
        print(motor)
        try:
            while self.serial.in_waiting:
                response = self.serial.readline().decode('utf-8', errors='ignore').strip()
                print(response)
                if motor == "FORT" and response.startswith("FORT_ANGLE:"):
                    return float(response.split(":")[1])
                if motor == "BASE" and response.startswith("BASE_ANGLE:"):
                    print('send')
                    return float(response.split(":")[1])
                    
        except serial.SerialException:
            print(f"Failed to retrieve {motor} angle. Reconnecting...")
            self.open_serial_connection()
            return None
    
    def set_target_angle(self,motor, angle):
        command = f"SET_{motor}:{angle}\n"
        self.send_serial_command(command)

    def reset_motor(self,motor):
        command = f"RESET_{motor}\n"
        self.send_serial_command(command)

    def reset_motor_switch(self,motor):
        command = f"RESET_MS_{motor}\n"
        self.send_serial_command(command)
        
        # Wait for the Arduino to reset or stabilize
        time.sleep(5)  # Adjust this delay based on how long the reset takes
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


class Cannon(threading.Thread):
    bus = None
    def __init__(self):
        super(Cannon, self).__init__()
        self.daemon = True
        self.arduino = Arduino()
        self.base = 0
        self.fort = 0
        self.cmd = None
        self.initRos()

    def initRos(self):
        rospy.init_node('cannon_node', anonymous=True)
        self.rate = rospy.Rate(10)
        rospy.Subscriber("/cannon", String, self.callback)

    def callback(self,data):
        msg = data.data.split(",")
        if len(msg) != 3:
            return
        self.cmd  = msg[0]
        if self.cmd not in ["reset_fort","reset_base"]:
            self.base = int(msg[1])
            self.fort = int(msg[2])
        #rospy.loginfo(rospy.get_caller_id() + "stall %s",self._stall)

    def run(self):
        b = True
        while not rospy.is_shutdown():
            if self.cmd == "reset_fort":
                self.arduino.reset_motor_switch("FORT")
                continue
            elif self.cmd == "reset_base":
                self.arduino.reset_motor_switch("BASE")
                continue
            print(self.fort,self.base)
            if b:
                b = False
                self.arduino.set_target_angle("FORT", self.fort)
            else:
                b = True
                self.arduino.set_target_angle("BASE", self.base)
            self.rate.sleep()

if __name__ == '__main__':
    try:
        Cannon = Cannon()
        Cannon.run()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
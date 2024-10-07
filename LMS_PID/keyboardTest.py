import serial
import time
from pynput import keyboard
import threading

# Serial Port Information
SERIAL_PORT = 'COM5'  # Update this to your port
BAUD_RATE = 57600
# Global Serial Object

class Arduino:
    def __init__(self):
        self.serial = None
        self.open_serial_connection()
        self.recvThread = threading.Thread(target=self.recv)
        self.recvThread.daemon = True
        self.recvThread.start()
        self.base_angle_input = 0
        self.base_target_angle = 0
        self.fort_target_angle = 0
        self.fort_angle_input = 0
        self.status = 0

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
            self.serial.flush()
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
        while True:
            if self.serial.in_waiting:
                response = self.serial.readline().decode('utf-8', errors='ignore').strip()
                response = response.split(",")
                self.status = int(response[0])
                self.base_angle_input = int(response[1])
                self.fort_angle_input = int(response[3])
                print(response)

if __name__ == '__main__':
    arduino = Arduino()
    currently_pressed = set()
    def on_release(key):
        try:
            currently_pressed.remove(key.char)
        except AttributeError:
            if key in [keyboard.Key.space, 
                       keyboard.Key.up, 
                       keyboard.Key.down, 
                       keyboard.Key.left, 
                       keyboard.Key.right]:
                currently_pressed.remove(key)
            pass
    def on_press(key):
        try:
            currently_pressed.add(key.char)
        except AttributeError:
            if key in [keyboard.Key.space, 
                       keyboard.Key.up, 
                       keyboard.Key.down, 
                       keyboard.Key.left, 
                       keyboard.Key.right]:
                currently_pressed.add(key)
            pass
    listener = keyboard.Listener(on_press=on_press, on_release=on_release)
    listener.start()
    b = True
    while True:
        if currently_pressed == set(['w']):
            arduino.fort_angle_input -= 1
        elif currently_pressed == set(['s']):
            arduino.fort_angle_input += 1
        elif currently_pressed == set(['a']):
            arduino.base_angle_input -= 2
        elif currently_pressed == set(['d']):
            arduino.base_angle_input += 2
        elif currently_pressed == set(['b']):
            print("reset base")
            arduino.reset_motor_switch("BASE")
            print("reset base done")
            continue
        elif currently_pressed == set(['f']):
            print("reset fort")
            arduino.reset_motor_switch("FORT")
            print("reset fort done")
            continue
        elif currently_pressed == set([keyboard.Key.space]):
            arduino.launch()
            time.sleep(0.1)
            print("launched")
            continue
        print(arduino.base_angle_input, arduino.fort_angle_input)
        if b:
            arduino.set_target_angle("BASE", arduino.base_angle_input)
            b = False
        else:
            arduino.set_target_angle("FORT", arduino.fort_angle_input)
            b = True
        time.sleep(0.01)
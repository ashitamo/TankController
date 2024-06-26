import can
import time


if __name__ == '__main__':
    bustype = 'socketcan'
    channel = 'can0'
    bus = can.Bus(interface=bustype, channel=channel, receive_own_messages=False)
    with open('data.txt', 'a', newline='') as fw:
        while True:
            msg = bus.recv()
            if msg.arbitration_id == 0x0363:
                print("363: ",file=fw)
                print("\tvsp",msg.data[0]+msg.data[1]*256,file=fw)
                print("\tvcu1",bin(msg.data[4]),file=fw)
                print("\tvcu2",bin(msg.data[5]),file=fw)
                print("\tsign",bin(msg.data[6]),file=fw)


            elif msg.arbitration_id == 0x0A3:
                print("A3: ",file=fw)
                print("\tstall",msg.data[0],file=fw)

            elif msg.arbitration_id == 0x067:
                print("67: ",file=fw)
                print("\tEPS Voltage",msg.data[1]/100*12,file=fw)
                print("\tSteer ang",msg.data[2]+msg.data[3]*256,file=fw)
                print("\tSteer torque",msg.data[4]+msg.data[5]*256,file=fw)
                print("\tSteer speed",msg.data[6]+msg.data[7]*256,file=fw)
            elif msg.arbitration_id == 0x167:
                print("167: ",file=fw)
                print("\tEPS assist",bin(msg.data[3]),file=fw)
                print("\tEPS status",bin(msg.data[7]),file=fw)
            elif msg.arbitration_id == 0x267:
                print("267: ",file=fw)
                print("\tsteer centeral position",msg.data[0]+msg.data[1]*256,file=fw)
                print("\tsteer left position",msg.data[2]+msg.data[3]*256,file=fw)
                print("\tsteer right position",msg.data[4]+msg.data[5]*256,file=fw)
            elif msg.arbitration_id == 0x101:
                print("101: ",file=fw)
                print("\tright speed",msg.data[0]+msg.data[1]*256+msg.data[2]*256*256+msg.data[3]*256*256*256,file=fw)
                print("\tleft speed",msg.data[4]+msg.data[5]*256+msg.data[6]*256*256+msg.data[7]*256*256*256,file=fw)
            elif msg.arbitration_id == 0x50:
                print("50: ",file=fw)
                print("\tthrottle",msg.data[0],file=fw)
            else:
                print(msg)

            time.sleep(0.1)


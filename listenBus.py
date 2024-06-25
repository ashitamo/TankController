import can



if __name__ == '__main__':
    bustype = 'socketcan'
    channel = 'can0'
    bus = can.Bus(interface=bustype, channel=channel, receive_own_messages=False)

    while True:
        msg = bus.recv()
        if msg.arbitration_id == 0x43f:
            print(bin(msg.data[4]))
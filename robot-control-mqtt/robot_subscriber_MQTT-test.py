import paho.mqtt.client as mqtt
import struct
import json
import matplotlib.pyplot as plt
import signal

TARGET_IDENTIFIER = 2

ip_addr = "10.53.49.161"
port = 1883
keepalive = 60

class SignalHandler():

    def __init__(self):
        self.state = False
        signal.signal(signal.SIGINT, self.change_state)

    def change_state(self, signum, frame):
        signal.signal(signal.SIGINT, signal.SIG_DFL)
        self.state = True

    def exit(self):
        return self.state


flag = SignalHandler()



def on_connect(client, userdata, flags, rc):
    print("Connected with result code", rc)
    client.subscribe("v2/robot/NRF_5/line", qos=1)
    client.subscribe("v2/robot/NRF_5/point", qos=0)



def on_message(client, userdata, msg):
    if (msg.topic == 'v2/robot/NRF_5/point'):
        (id, dx, dy, dtheta, ir1x, ir1y, ir2x, ir2y, ir3x, ir3y, ir4x, ir4y, valid) = struct.unpack('<BhhhhhhhhhhhB', msg.payload)
        print("dx: {}, dy: {}, dtheta: {}, ir: [({}, {}), ({}, {}), ({}, {}), ({},{})], valid: {}".format(dx, dy, dtheta, ir1x, ir1y, ir2x, ir2y, ir3x, ir3y, ir4x, ir4y, valid))
        if (dx == 0 and dy == 0 and dtheta == 0 and ir1x == 100 and ir1y == 100 and ir2x == 100 and ir2y == 100 and ir3x == 100 and ir3y == 100 and ir4x == 100 and ir4y == 100 and valid == 15):
            client.num_rx_points += 1

        
    elif (msg.topic == 'v2/robot/NRF_5/line'):
        (id, dx, dy, dtheta, start_x, start_y, end_x, end_y) = struct.unpack('<Bhhhhhhh', msg.payload)
        print("dx: {}, dy: {}, dtheta: {}. line start: ({},{}), end: ({},{})".format(dx, dy, dtheta, start_x, start_y, end_x, end_y))
        if (dx == 0 and dy == 0 and dtheta == 0 and start_x == -500 and start_y == 500 and end_x == 500 and end_y == 500):
            client.num_rx_lines += 1
        
        




client = mqtt.Client(client_id="robot-subscriber")
client.connect(ip_addr, port, keepalive)
client.on_connect = on_connect
client.on_message = on_message
client.num_rx_lines = 0
client.num_rx_points = 0

client.loop_start()

while(True):
    if flag.exit():
        print("###### STATS #######")
        print("Received lines: {}".format(client.num_rx_lines))
        print("Received points: {}".format(client.num_rx_points))
        exit()


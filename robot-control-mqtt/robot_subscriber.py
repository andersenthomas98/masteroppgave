import paho.mqtt.client as mqtt
import struct
import json
import matplotlib.pyplot as plt
import signal

TARGET_IDENTIFIER = 2

ip_addr = "10.53.50.100"
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


line_file = open("line_log.txt", "w")
point_file = open("point_log.txt", "w")
dbscan_file = open("dbscan_log.txt", "w")
iepf_file = open("iepf_log.txt", "w")
common_point_file = open("common_point_log.txt", "w")
mse_file = open("mse_log.txt", "w")

def on_connect(client, userdata, flags, rc):
    print("Connected with result code", rc)
    client.subscribe("v2/robot/NRF_5/controller", qos=0)
    client.subscribe("v2/robot/NRF_5/coordinate", qos=0)
    client.subscribe("v2/robot/NRF_5/DBSCAN", qos=0)
    client.subscribe("v2/robot/NRF_5/IEPF", qos=0)
    client.subscribe("v2/robot/NRF_5/line", qos=0)
    client.subscribe("v2/robot/NRF_5/adv", qos=0)
    client.subscribe("v2/robot/NRF_5/point", qos=0)
    client.subscribe("v2/robot/NRF_5/MSE", qos=0)


def on_message(client, userdata, msg):
    #print(msg.topic + ": " + str(msg.payload))
    if (msg.topic == 'v2/robot/NRF_5/controller'):
        (t, x, y, theta, left_u, right_u) = struct.unpack('<ffffff', msg.payload)

        print("t: {}, x: {}, y: {}, theta: {}, left_u: {}, right_u: {}".format(t,x,y,theta, left_u, right_u))


    elif (msg.topic == 'v2/robot/NRF_5/coordinate'):
        (x, y) = struct.unpack('<hh', msg.payload)
 
        print("x: {}, y: {}".format(x,y))

    elif (msg.topic == 'v2/robot/NRF_5/DBSCAN'):
        #print(struct.unpack('<BBhhh'))
        (cluster_id, x, y) = struct.unpack('<bhh', msg.payload)
        print(" DBSCAN cluster_id: {}, x: {}, y: {}".format(cluster_id, x, y))
        entry = json.dumps({"id": cluster_id, "x": x, "y": y}) + '\n'
        dbscan_file.write(entry)

    elif (msg.topic == 'v2/robot/NRF_5/point'):
        #print(struct.unpack('<BBhhh'))
        (cluster_id, x, y) = struct.unpack('<bhh', msg.payload)
        print(" point x: {}, y: {}".format(x, y))
        entry = json.dumps({"x": x, "y": y}) + '\n'
        common_point_file.write(entry)
    
    elif (msg.topic == 'v2/robot/NRF_5/IEPF'):
        #print(struct.unpack('<BBhhh'))
        (cluster_id, x, y) = struct.unpack('<bhh', msg.payload)
        print("IEPF cluster_id: {}, x: {}, y: {}".format(cluster_id, x, y))
        entry = json.dumps({"id": cluster_id, "x": x, "y": y}) + '\n'
        iepf_file.write(entry)


    elif (msg.topic == 'v2/robot/NRF_5/line'):
        #(id, x, y, theta, start_x, start_y, end_x, end_y) = struct.unpack('<Bhhhhhhh', msg.payload)
        #print("id: {}, x: {}, y: {}, theta: {}, start: ({},{}), end: ({},{})".format(id, x, y, theta, start_x, start_y, end_x, end_y))
        #entry = json.dumps({"x": x, "y": y, "theta": theta, "start": {"x": start_x, "y": start_y}, "end": {"x": end_x, "y": end_y}}) + '\n'
        (id, start_x, start_y, end_x, end_y, sigma_r2, sigma_theta2, sigma_rtheta) = struct.unpack('<Bhhhhfff', msg.payload)
        print("line start: ({},{}), end: ({},{}), R: [[{}, {}], [{}, {}]]".format(id, start_x, start_y, end_x, end_y, sigma_r2, sigma_rtheta, sigma_rtheta, sigma_theta2))
        entry = json.dumps({"start": {"x": start_x, "y": start_y}, "end": {"x": end_x, "y": end_y}, "sigma_r2": sigma_r2, "sigma_theta2": sigma_theta2, "sigma_rtheta": sigma_rtheta}) + '\n'
        line_file.write(entry)

    elif (msg.topic == 'v2/robot/NRF_5/MSE'):
        (id, start_x, start_y, end_x, end_y, sigma_r2, sigma_theta2, sigma_rtheta) = struct.unpack('<Bhhhhfff', msg.payload)
        print("MSE start: ({},{}), end: ({},{}), R: [[{}, {}], [{}, {}]]".format(id, start_x, start_y, end_x, end_y, sigma_r2, sigma_rtheta, sigma_rtheta, sigma_theta2))
        entry = json.dumps({"start": {"x": start_x, "y": start_y}, "end": {"x": end_x, "y": end_y}, "sigma_r2": sigma_r2, "sigma_theta2": sigma_theta2, "sigma_rtheta": sigma_rtheta}) + '\n'
        mse_file.write(entry)

    elif (msg.topic == 'v2/robot/NRF_5/adv'):
        #print(msg.payload)
        (id, x, y, theta, ir1x, ir1y, ir2x, ir2y, ir3x, ir3y, ir4x, ir4y, valid) = struct.unpack('<B11hB', msg.payload)
        print("id, {}, x: {}, y: {}, theta: {}, ir1: ({},{}), ir2: ({},{}), ir3: ({},{}), ir4: ({},{})".format(id, x, y, theta, ir1x, ir1y, ir2x, ir2y, ir3x, ir3y, ir4x, ir4y))
        entry = json.dumps({"x": x, "y": y, "theta": theta, "ir1": {"x": ir1x, "y": ir1y}, "ir2": {"x": ir2x, "y": ir2y}, "ir3": {"x": ir3x, "y": ir3y}, "ir4": {"x": ir4x, "y": ir4y}}) + '\n'
        point_file.write(entry)



client = mqtt.Client(client_id="robot-subscriber")
client.connect(ip_addr, port, keepalive)
client.on_connect = on_connect
client.on_message = on_message

client.loop_start()

while(True):
    if flag.exit():
        print("Closing files")
        point_file.close()
        line_file.close()
        dbscan_file.close()
        iepf_file.close()
        common_point_file.close()
        mse_file.close()
        exit()


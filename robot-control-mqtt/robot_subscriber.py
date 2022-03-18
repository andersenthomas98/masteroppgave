import paho.mqtt.client as mqtt
import struct
import json

TARGET_IDENTIFIER = 2

ip_addr = "10.53.50.158"
port = 1883
keepalive = 60


file = open("log.json", 'w')

def on_connect(client, userdata, flags, rc):
    print("Connected with result code", rc)
    client.subscribe("v2/robot/NRF_5/controller", qos=0)

def on_message(client, userdata, msg):
    #print(msg.topic + ": " + str(msg.payload))
    (t, x, y, theta, left_u, right_u) = struct.unpack('<ffffff', msg.payload)

    log_entry = {"t": t, "x": x, "y": y, "theta": theta, "left_u": left_u, "right_u": right_u}
    json.dump(log_entry, file)

    print("t: {}, x: {}, y: {}, theta: {}, left_u: {}, right_u: {}".format(t,x,y,theta, left_u, right_u))

client = mqtt.Client()
client.connect(ip_addr, port, keepalive)
client.on_connect = on_connect
client.on_message = on_message

client.loop_start()

while(True):
    try:
        pass
    except KeyboardInterrupt:
        file.close()
        exit()


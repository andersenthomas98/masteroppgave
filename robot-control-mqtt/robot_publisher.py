import paho.mqtt.client as mqtt
import struct

TARGET_IDENTIFIER = 2

ip_addr = "192.168.0.109"
port = 1883
keepalive = 60

def on_connect(client, userdata, flags, rc):
    print("Connected with result code", rc)
    client.subscribe("v2/robot/NRF_5/controller", qos=0)

def on_message(client, userdata, msg):
    print(msg.topic + ": " + str(msg.payload))


client = mqtt.Client()
client.connect(ip_addr, port, keepalive)
client.on_connect = on_connect

client.loop_start()

while(True):
    cmd = input()
    payload = struct.pack('<Bhh', TARGET_IDENTIFIER, int(cmd), 0)
    print(struct.unpack('<Bhh', payload))
    client.publish("v2/server/NRF_5/cmd", payload)

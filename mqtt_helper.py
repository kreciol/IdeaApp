#!/usr/bin/ python

import paho.mqtt.client as mqtt

mqttc = None
connected = False

def on_connect(client, data, flags, rc):
    global connected
    print()
    print('Connected, rc: ' + str(rc))
    connected = True

def on_disconnect(client, userdata, rc):
    global connected
    connected = False
    if rc != 0:
        print("Unexpected disconnection.")
    else:
        print("Disconnected!!!")

def connectToMqttServer(server):
    global mqttc
    mqttc = mqtt.Client(clean_session=True)
    mqttc.on_connect = on_connect
    mqttc.on_disconnect = on_disconnect
    mqttc.connect(server, 1883)
    mqttc.loop_start()
    return mqttc

def publishMessage(topic, message):
    mqttc.publish(topic, message)

def isMqttConnected():
    return connected;
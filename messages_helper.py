#!/usr/bin/ python

from mqtt_helper import connectToMqttServer
from mqtt_helper import publishMessage
from mqtt_helper import isMqttConnected

import json

MQTT_TOPIC_TEST = "ideaapp/masters/test"

mqttc = None

def connectToMqtt(server):
    mqttc = connectToMqttServer(server)

def getCoordinates(x, y):
    coordinates = dict()
    coordinates['x'] = x
    coordinates['y'] = y
    return coordinates;

def initMessageByType(type):
    obj = dict()
    obj['type'] = type
    return obj

def send(message):
    str = json.dumps(message, ensure_ascii=False)
    print(str)
    publishMessage(MQTT_TOPIC_TEST, str)

def moveTo(x, y):
    obj = initMessageByType('move')
    obj['data'] = getCoordinates(x, y)
    send(obj)

def lineTo(x, y):
    obj = initMessageByType('line')
    obj['data'] = getCoordinates(x, y)
    send(obj)

def clear():
    obj = initMessageByType('clear')
    send(obj)

def isConnected():
    return isMqttConnected();
import time
import os
import string
import paho.mqtt.client as mqtt
import requests
import random

RPi_HOST = "10.0.0.17"
localBroker = RPi_HOST		# Local MQTT broker
localPort = 1883			# Local MQTT port
UTC_OFFSET = 3   # hours of differenc between UTC and local (Jerusalem) time
CLIENT_ID= b"FAKELogger"
localTimeOut = 120			# Local MQTT session timeout


def pushSample(sample, topic):
    global client
    client.publish(topic, str(sample))



if __name__ == "__main__":
    #Generic Init
    print ("Initializing...")

    #MQTT configs
    client = mqtt.Client()
    try:
        time.sleep(3)
        client.connect(localBroker, localPort, localTimeOut)
        print("Connected to {}".format(localBroker))
        print ("Initializing Done")
    except:
        print("failed to connect to RPi. waiting 5sec and resetting")
        time.sleep(5)

    while True:
        val = random.randint(50,100)
        pushSample(val,"/sensor_"+CLIENT_ID+"/chipa/humidity")
        print ("Publish: ",val,"to  /sensor/chipa/humidity")
        time.sleep(1)
        val = random.randint(15,40)
        pushSample(val,"/sensor_"+CLIENT_ID+"/chipa/temperature")
        print ("Publish: ",val,"to  /sensor/chipa/temperature")
        time.sleep(5)


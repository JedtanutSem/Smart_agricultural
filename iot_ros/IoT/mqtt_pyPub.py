import paho.mqtt.client as mqtt
import json
import time
#import rospy
i = 0
B = 5
host = "10.66.11.150"
port = 1883
client = mqtt.Client()
client.connect(host)
#data = "["Test"]"

while i != 10:
    MQTT_MSG = json.dumps({"A": i, "C": 7, "G": "44.5", "petalWidth": "1.5"});
    client.publish("data",MQTT_MSG)
    #rospy.loginfo(MQTT_MSG)
    time.sleep(1)
    i = i+1
    if(i == 9):
        i = 0

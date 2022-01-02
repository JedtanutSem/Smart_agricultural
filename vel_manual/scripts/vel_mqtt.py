#!/usr/bin/env python

import paho.mqtt.client as mqttClient
import time
import rospy
from std_msgs.msg import String
mqtt_data = 0

def on_connect(client, userdata, flags, rc):
    if rc == 0:
        print("Connected to broker")
        #client.subscribe("date")
        client.subscribe("date_1")
        #client.subscribe("topic/3")
        #client.subscribe("topic/4")

    else:
        print("Connection failed")

def on_message(client, userdata, message):

    #global mqtt_data
    print("Message received : "  + str(message.payload) + " on " + message.topic)
    #mqtt_data = message.payload



rospy.init_node('vel_mqtt', anonymous = True)
rate = rospy.Rate(10)
broker_address= "localhost"
port = 1883

client = mqttClient.Client("Python")
client.on_connect= on_connect
client.subscribe([("topic/1", 0), ("topic/2", 0), ("topic/3", 0),("topic/4", 0)])
client.on_message= on_message

client.connect(broker_address, port=port)
client.loop_start()


try:
    while not rospy.is_shutdown():
        #time.sleep(1)
        rate.sleep()
        #rospy.loginfo(mqtt_data)

except KeyboardInterrupt:
    print("exiting")
    client.disconnect()
    client.loop_stop()



"""
while not rospy.is_shutdown():

    client.loop_start()
    client.subscribe([("date", 0), ("date_1", 0)])
    #client.subscribe("date")
    #client.on_message=on_message
    #vel_x = mqtt_data
    #client.subscribe("date_1")
    client.on_message=on_message
    #ang_z = mqtt_data
    #time.sleep(0.5)
    #client.loop_stop()
    #rospy.loginfo(on_message)
    #rospy.loginfo(str(vel_x)+" "+str(ang_z))
    rospy.loginfo(mqtt_data)
    #pub.publish(mqtt_data)
    rate.sleep()
"""

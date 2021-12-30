#!/usr/bin/env python

import paho.mqtt.client as mqtt
import time
import rospy
from std_msgs.msg import String
mqtt_data = ""

def on_message(client, userdata, message):
    global mqtt_data
    #print("received message: " ,str(message.payload.decode("utf-8")))
    mqtt_data = str(message.payload.decode("utf-8"))

    #rospy.loginfo("received message: " ,str(message.payload.decode("utf-8")))


rospy.init_node('mqtt_sub', anonymous = True)
mqttBroker ="localhost"

client = mqtt.Client("python_sub")
client.connect(mqttBroker)
rate = rospy.Rate(10)
pub = rospy.Publisher('mqtt_subSend', String, queue_size = 10)
while not rospy.is_shutdown():

    client.loop_start()

    client.subscribe("date")
    client.on_message=on_message

    #time.sleep(0.5)
    #client.loop_stop()
    #rospy.loginfo(on_message)
    #rospy.loginfo(mqtt_data)
    pub.publish(mqtt_data)
    rate.sleep()

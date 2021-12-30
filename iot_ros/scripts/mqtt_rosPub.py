#!/usr/bin/env python

import paho.mqtt.client as mqtt
import json
import time
import rospy
from geometry_msgs.msg import Twist
i = 0
B = 5
vel_x = 0
ang_z = 0
host = "10.66.11.150"
port = 1883
client = mqtt.Client()
client.connect(host)
#data = "["Test"]"

def vel_blk(vel):
    global vel_x
    global ang_z
    vel_x = vel.linear.x
    ang_z = vel.angular.z
    #rospy.loginfo(vel_x)


rospy.init_node('mqtt_vel_pub', anonymous = True)
rospy.Subscriber('cmd_vel', Twist, vel_blk)
rate = rospy.Rate(10)

while not rospy.is_shutdown():
    MQTT_MSG = json.dumps({"vel_x": vel_x, "ang_z": ang_z});
    #MQTT_MSG = vel_x
    client.publish("vel_x",vel_x)
    client.publish("ang_z",ang_z)
    #rospy.loginfo(MQTT_MSG)
    #time.sleep(1
    rate.sleep()

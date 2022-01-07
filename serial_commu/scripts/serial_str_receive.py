#!/usr/bin/env python

import serial
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import time
import json
max_linear_speed = 0.5
max_angular_speed = 1

if __name__ == "__main__":

    arduino = serial.Serial('/dev/ttyUSB0', 9600, timeout=.1)

    #rospy.Subscriber('chatter', String, callback)
    pub = rospy.Publisher('serial_read', String, queue_size=1)
    pub_vel = rospy.Publisher('cmd_vel',Twist,queue_size=1000)
    rospy.init_node('read_ser', anonymous=True)
    rate = rospy.Rate(200) # 10hz

    while not rospy.is_shutdown():

        #str = '%s,%s\n' %(get_a,get_b)
        #rospy.loginfo(str)
        #arduino.write(str)
        data = arduino.readline()[:-2]

        if data:
            serial_str = data
            serial_split = serial_str.split(',')
            #rospy.loginfo(len(serial_split))
            try:
                if len(serial_split) == 2:
                    x_linear_val = serial_split[0]
                    x_linear_val = int(x_linear_val)
                    x_linear_vel =  max_linear_speed * (x_linear_val / 100)

                    z_angular_val = serial_split[1]
                    z_angular_val = int(z_angular_val)
                    z_angular_vel = max_angular_speed * (z_angular_val / 100)


                    msg = Twist()
                    msg.linear.x = x_linear_vel
                    msg.angular.z = z_angular_vel
                    pub_vel.publish(msg)
            except:
                pass

            rospy.loginfo(serial_str)
            pub.publish(serial_str)
        rate.sleep()

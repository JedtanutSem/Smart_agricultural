#!/usr/bin/env python

import serial
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import time
import json
max_linear_speed = 0.5
max_angular_speed = 1
vel_x = 0

def vel_clbk(data):
    global vel_x
    vel_x = data.linear.x


    #pass

if __name__ == "__main__":

    rospy.init_node('read_ser', anonymous=True)
    pub = rospy.Publisher('serial_read', String, queue_size=1)
    pub_vel = rospy.Publisher('cmd_vel',Twist,queue_size=10)
    rospy.Subscriber("cmd_vel",Twist, vel_clbk)
    rate = rospy.Rate(200) # 10hz
    try:
        arduino = serial.Serial('/dev/ttyUSB0', 9600, timeout=.1)
        #rospy.Subscriber('chatter', String, callback)
        #rospy.loginfo("test")

        while not rospy.is_shutdown():
            #ospy.loginfo("loop")

            str = '%s,%s,%s,%s,%s\n' %(2,2,2,2,vel_x)
            #rospy.loginfo(str)
            arduino.write(str)
            data = arduino.readline()[:-2]
            rospy.loginfo(data)
    # left,right
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
                        #pub_vel.publish(msg)
                except:
                    pass

                rospy.loginfo(serial_str)
                pub.publish(serial_str)
    except:
        rospy.loginfo("Serial Fail")

    rate.sleep()

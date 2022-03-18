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

pwmL = 0
pwmR = 0
dirL = 0
dirR = 0

def vel_clbk(data):
    global vel_x
    vel_x = data.linear.x
def pwm_get_clbk(data):
    global pwmL, pwmR, dirL, dirR
    pwm_str = data.data
    pwm_list = pwm_str.split(',')
    pwmL = float(pwm_list[0])
    dirL = int(pwm_list[1])
    pwmR = float(pwm_list[2])
    dirR = int(pwm_list[3])
    print(pwm_str)

    #pass

if __name__ == "__main__":

    rospy.init_node('read_ser', anonymous=True)
    pub = rospy.Publisher('serial_read_arduino', String, queue_size=1)
    pub_vel = rospy.Publisher('cmd_vel',Twist,queue_size=10)
    rospy.Subscriber("cmd_vel",Twist, vel_clbk)
    rospy.Subscriber('pwm_to_controller', String, pwm_get_clbk)
    rate = rospy.Rate(150) # 10hz
    try:
        arduino = serial.Serial('/dev/ttyUSB0', 9600, timeout=.1)
        #rospy.Subscriber('chatter', String, callback)
        #rospy.loginfo("test")

        while not rospy.is_shutdown():
            #ospy.loginfo("loop")
            str = '%s,%s,%s,%s\n' %(pwmL,dirL,pwmR,dirR)#pwm_L, dir_L, pwm_R, dir_R
            #1--> forward , 0--> backward

            #rospy.loginfo(str)
            arduino.write(str)
            data = arduino.readline()[:-2]
            #ospy.loginfo(str)
    # left,right
            if data:
                serial_str = data
                serial_split = serial_str.split(',')
                #rospy.loginfo(len(serial_split))
                try:
                    if len(serial_split) == 2:
                        L_count_str = serial_split[0]
                        L_count_int = int(L_count_str)
                        #x_linear_vel =  max_linear_speed * (x_linear_val / 100)

                        R_count_str = serial_split[0]
                        R_count_int = int(R_count_str)
                        #z_angular_vel = max_angular_speed * (z_angular_val / 100)
#
                        """
                        msg = Twist()
                        msg.linear.x = x_linear_vel
                        msg.angular.z = z_angular_vel
                        #pub_vel.publish(msg)
                        """
                except:
                    pass

                #rospy.loginfo(serial_str)
                pub.publish(serial_str)
                rate.sleep()
    except Exception as e:
        rospy.loginfo("Serial Fail")
        print e
